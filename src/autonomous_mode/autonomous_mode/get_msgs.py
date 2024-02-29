import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleImu,SensorGps,VehicleOdometry,VehicleAttitude
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,NavSatFix
from std_msgs.msg import Header



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
        self.odometry_publisher = self.create_publisher(
            Odometry, 'odom', qos_profile)
        self.Imu_publisher = self.create_publisher(
            Imu, '/imu', qos_profile)
        self.gps_publisher = self.create_publisher(
            NavSatFix, 'gps/fix', qos_profile)

        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_imu_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_imu', self.vehicle_imu_callback, qos_profile)
        self.vehicle_imu_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_callback, qos_profile)
        self.timer=self.create_timer(0.2,self.timer_callback)
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_imu=VehicleAttitude()
        self.vehicle_sensor = SensorGps()
        
        # # Create a timer to publish control commands
        # self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_odometry_callback(self,msg):
        self.vehicle_odometry=msg

    def vehicle_gps_callback(self,msg):
        self.vehicle_sensor=msg
    
    def vehicle_imu_callback(self,msg):
        self.vehicle_imu=msg



    def vehicle_imu_publisher(self):
        imu=Imu()
        imu.header.frame_id="base_link"
        imu.orientation.x=float(self.vehicle_imu.q[0])
        imu.orientation.y=float(self.vehicle_imu.q[1])
        imu.orientation.z=float(self.vehicle_imu.q[2])
        imu.orientation.w=float(self.vehicle_imu.q[3])
        self.Imu_publisher.publish(imu)

    def vehicle_odometry_publisher(self):
        msg=Odometry()
        msg.header.frame_id='world_frame'
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.child_frame_id='base_link'
        msg.pose.pose.position.x=float(self.vehicle_odometry.position[1])
        msg.pose.pose.position.y=float(self.vehicle_odometry.position[0])
        msg.pose.pose.position.z=-float(self.vehicle_odometry.position[2])
        msg.twist.twist.linear.x=float(self.vehicle_odometry.velocity[0])
        msg.twist.twist.linear.y=-float(self.vehicle_odometry.velocity[1])
        msg.twist.twist.linear.z=-float(self.vehicle_odometry.velocity[2])

        self.odometry_publisher.publish(msg)

        # self.vehicle_odometry.twist=msg.velocity_variance


    def vehicle_gps_publisher(self):
        gps_cordinate=NavSatFix()
        gps_cordinate.header=Header()
        gps_cordinate.header.stamp =self.get_clock().now().to_msg()
        gps_cordinate.header.frame_id = 'base_link'  # Update with your GPS frame
        gps_cordinate.position_covariance_type=0
        gps_cordinate.latitude=self.vehicle_sensor.latitude_deg
        gps_cordinate.longitude=self.vehicle_sensor.longitude_deg
        gps_cordinate.altitude=self.vehicle_sensor.altitude_msl_m	
        self.gps_publisher.publish(gps_cordinate)


    def timer_callback(self):

        self.vehicle_gps_publisher()
        self.vehicle_imu_publisher()
        self.vehicle_odometry_publisher()

def main(args=None) -> None:
    print('Collecting_msgs')
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
