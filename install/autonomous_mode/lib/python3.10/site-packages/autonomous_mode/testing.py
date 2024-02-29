import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus,VehicleAttitudeSetpoint,VehicleAttitude
from geometry_msgs.msg import Quaternion
import math

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude', qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position


    def euler_from_quaternion(self,q: Quaternion):
        """
        Convert a quaternion into euler angles
        taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        """
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def vehicle_attitude_callback(self,msg):
        vehicle_attitude=Quaternion()
        vehicle_attitude.x=float(msg.q[0])
        vehicle_attitude.y=float(msg.q[1])
        vehicle_attitude.z=float(msg.q[2])
        vehicle_attitude.w=float(msg.q[3])
        
        self.drone_angle,_,_=self.euler_from_quaternion(vehicle_attitude)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_attitude_setpoint(self,yaw_angle:float):
        msg=VehicleAttitudeSetpoint()
        msg.roll_body=0.0
        msg.pitch_body=0.0
        msg.yaw_body=yaw_angle
        msg.yaw_sp_move_rate=0.05
        msg.thrust_body[0]=0.0
        msg.thrust_body[1]=0.0
        msg.thrust_body[2]=-0.1
        msg.q_d
        self.vehicle_attitude_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        
        # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.yawspeed=0.1
        
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_velocity_setpoint(self, x: float, y: float, z: float,yaw_angle:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position=[0.0,0.0,self.takeoff_height]
        msg.velocity=[x,y,z]
        # msg.position=[0.0,0.0,self.takeoff_height]
        msg.yaw = yaw_angle  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.yawspeed=0.05
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z,self.drone_angle]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            
            self.publish_velocity_setpoint(0.0, 0.0, -0.7,1.57)
            print(self.drone_angle)

        elif self.vehicle_local_position.z <= self.takeoff_height and self.drone_angle>=0.2:
            print(self.drone_angle)
            self.publish_position_setpoint(self.vehicle_local_position.x+10,0.0,0.0)
        
        elif self.vehicle_local_position.z <= self.takeoff_height:
            print(self.drone_angle)
            self.publish_position_setpoint(1.0,0.0,0.0,1.57)
        
        
            

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

# abs(abs(self.takeoff_height)-abs(self.vehicle_local_position.z))/2