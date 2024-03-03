import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus,VehicleAttitudeSetpoint,VehicleAttitude,SensorGps
from geometry_msgs.msg import Quaternion
import math
from rclpy.executors import MultiThreadedExecutor,SingleThreadedExecutor
import threading
import time
import math,utm

class SharedVariables:
    def __init__(self):
        self.vehicle_local_pos = VehicleLocalPosition()
        self.vehicle_gps_pos = SensorGps()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_status_ = VehicleStatus()
        self.offboard_setpoint_counter = 0.0

shared_variables = SharedVariables()





class OffboardControl(Node):
    def __init__(self, shared_variables):
        super().__init__('offboard_control_takeoff_and_land')
        self.shared_variables = shared_variables

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_callback, qos_profile)
        # Initialize variables
        self.offboard_setpoint_counter = 0.0
        # threading.Thread(target=self.timer_callback_1).start()
        self.vehicle_gps=SensorGps()
        self.local_pos=VehicleLocalPosition()
        self.vehicle_attitide=VehicleAttitude()
        self.vehicle_stat=VehicleStatus()
        # # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback_1)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.shared_variables.vehicle_local_pos = vehicle_local_position


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
        self.shared_variables.vehicle_status_= vehicle_status
    
    def vehicle_gps_callback(self,vehicle_gps):
        self.shared_variables.vehicle_gps_pos= vehicle_gps

    def vehicle_attitude_callback(self,msg):
        
        
        self.shared_variables.vehicle_attitude=msg


    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")


    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = True
        msg.body_rate = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)


    
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
        print("Publishing command")

    def timer_callback_1(self) -> None:

        """Callback function for the timer."""
        
        self.publish_offboard_control_heartbeat_signal()
        if self.shared_variables.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()

        if self.shared_variables.offboard_setpoint_counter < 11:

            self.shared_variables.offboard_setpoint_counter += 1

        






class navigating_point(Node):
    def __init__(self, shared_variables):
        super().__init__('navigation_task')
        self.shared_variables = shared_variables
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Create publishers
        self.vehicle_attitude_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.drone_curr_angle=self.get_drone_angle()
        self.take_off_h=-5.0
        self.takeoff_done=False
        self.reached=False
        self.timer = self.create_timer(0.3, self.timer_callback_2)
        self.get_data=False
        self.yaw=False
        self.target_pos_x_ned=self.shared_variables.vehicle_local_pos.x
        self.target_pos_y_ned=self.shared_variables.vehicle_local_pos.y
        self.x_ned=0.0
        self.y_ned=0.0
        self.z_ned=0.0
        self.yaw_ned=self.drone_curr_angle
        # threading.Thread(target=self.timer_callback_2).start()

    def calculate_distance_and_angle(self,utm1,utm2):
        # UTM coordinates are in the format (easting, northing, zone number, zone letter)
        easting1, northing1, _, _ = utm1
        easting2, northing2, _, _ = utm2

        # Calculate distance using Pythagorean theorem
        distance = math.sqrt((easting2 - easting1)**2 + (northing2 - northing1)**2)

        # Calculate angle using arctan2
        angle = math.atan2(northing2 - northing1, easting2 - easting1)

        return distance, angle
    
    def convert_utm(self,lat:float,lon:float):
        utm_coordinate1 = utm.from_latlon(self.shared_variables.vehicle_gps_pos.latitude_deg, self.shared_variables.vehicle_gps_pos.longitude_deg)
        utm_coordinate2 = utm.from_latlon(lat, lon)
        distance,angle=self.calculate_distance_and_angle(utm_coordinate1,utm_coordinate2)
        return distance,angle
    

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
    


    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

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

    def takeoff(self,take_off_h:float):
        current_x=self.shared_variables.vehicle_local_pos.x
        current_y=self.shared_variables.vehicle_local_pos.x
        current_z=self.shared_variables.vehicle_local_pos.z
        print(current_z)
        if abs(abs(current_z)-abs(take_off_h))>0.2:
            self.publish_pos_vel_setpoint(current_x,current_y,self.take_off_h,0.0,0.0,self.take_off_h/10,self.drone_curr_angle)
            return False
        else:
            return True


    def publish_pos_vel_setpoint(self, x: float, y: float, z: float,x_vel:float,y_vel:float,z_vel:float,yaw_angle:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity=[x_vel,y_vel,z_vel]
        msg.yaw=yaw_angle
        msg.yawspeed=-0.2
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        
        
        
        self.trajectory_setpoint_publisher.publish(msg)
        
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")


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
        print("Publishing command")

    def publish_attitude_setpoint(self,yaw_angle:float):
        msg=VehicleAttitudeSetpoint()
        msg.roll_body=0.0
        msg.pitch_body=0.0
        msg.yaw_body=yaw_angle
        msg.yaw_sp_move_rate=0.05
        while abs(abs(self.drone_curr_angle)-abs(yaw_angle))<0.2:
            self.vehicle_attitude_publisher.publish(msg)

    def get_drone_angle(self):
        quat=Quaternion()
        quat.x=float(self.shared_variables.vehicle_attitude.q[0])
        quat.y=float(self.shared_variables.vehicle_attitude.q[1])
        quat.z=float(self.shared_variables.vehicle_attitude.q[2])
        quat.w=float(self.shared_variables.vehicle_attitude.q[3])

        drone_angle,_,_=self.euler_from_quaternion(quat)

        return drone_angle
    
    def navigate_to_point(self,target_pos_x:float,target_pos_y:float):
        current_x=self.shared_variables.vehicle_local_pos.x
        current_y=self.shared_variables.vehicle_local_pos.y
        print(abs(abs(current_x)-abs(target_pos_x)))
        print(abs(abs(current_y)-abs(target_pos_y)))
        
        
        if abs(abs(current_x)-abs(target_pos_x))<=0.2 and abs(abs(current_y)-abs(target_pos_y))<=0.2:
            return True
        else:
            self.publish_pos_vel_setpoint(target_pos_x,target_pos_y,self.take_off_h,0.0,0.0,0.0,self.drone_curr_angle)
            return False
        
        
    def get_conversion_enu_ned(self,x:float,y:float,z:float):

        return y,x,-z
    
    def get_yaw_angle(self,yaw_angle:float):

        return 1.57-yaw_angle
    
    def get_targets_ned(self,lat:float,lon:float):
        dist_wp,angle_wp=self.convert_utm(lat,lon)
        target_pos_x_enu=self.shared_variables.vehicle_local_pos.x+dist_wp*math.cos(angle_wp)
        target_pos_y_enu=self.shared_variables.vehicle_local_pos.x+dist_wp*math.sin(angle_wp)
        target_pos_x_ned,target_pos_y_ned,_=self.get_conversion_enu_ned(target_pos_x_enu,target_pos_y_enu,self.take_off_h)
        return target_pos_x_ned,target_pos_y_ned
        

    def timer_callback_2(self) -> None:
        """Callsback function for the timer."""
        
        self.drone_curr_angle=self.get_drone_angle()
        if self.shared_variables.vehicle_status_.nav_state==14 and self.shared_variables.vehicle_status_.arming_state==1:
            self.arm()

        if  self.takeoff_done==False and self.shared_variables.vehicle_status_.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            

            self.takeoff_done=self.takeoff(self.take_off_h)
            
            

        elif self.shared_variables.vehicle_local_pos.z <= self.take_off_h and self.reached==False and self.shared_variables.vehicle_status_.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            
            if not self.get_data:
                
                self.target_pos_x_ned,self.target_pos_y_ned=self.get_targets_ned(47.39779806134497, 8.545671128868838)
                self.get_data=True


            if not self.reached:
                print(f"{self.target_pos_x_ned},{self.target_pos_y_ned}")
                self.reached=self.navigate_to_point(self.target_pos_x_ned,self.target_pos_y_ned)
                print(self.reached)
            
        elif self.reached:

            self.publish_vehicle_command(23)
            
        
        
                
            
            
            
                

        





def main(args=None) -> None:

    rclpy.init(args=args)
    try:

        
        Navigator= navigating_point(shared_variables)
        OffboardControl_mode= OffboardControl(shared_variables)
        executor = MultiThreadedExecutor()
        executor.add_node(OffboardControl_mode)
        executor.add_node(Navigator)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            Navigator.destroy_node()
            OffboardControl_mode.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

# abs(abs(self.takeoff_height)-abs(self.vehicle_local_position.z))/2