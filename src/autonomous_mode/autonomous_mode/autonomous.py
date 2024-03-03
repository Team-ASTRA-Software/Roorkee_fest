import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus,SensorGps,VehicleAttitudeSetpoint,VehicleAttitude,VehicleImu
import math,utm
# from utilities import degrees_to_radians,euler_from_quaternion
from geometry_msgs.msg import Quaternion
from rclpy.time import Time,Duration
from rclpy.executors import MultiThreadedExecutor
import time
waypoints=[(47.39779806134497,  8.545671128868838),(47.39779371134151,8.545604635710545)]



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
        self.vehicle_attitude_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude', qos_profile)
        
        

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_callback, qos_profile)
        
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        # self.vehicle_imu_subscriber = self.create_subscription(
        #     VehicleImu, '/fmu/out/vehicle_imu', self.vehicle_imu_callback, qos_profile)
        # self.vehicle_odometry_subscriber = self.create_subscription(
        #     VehicleImu, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        


        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.vehicle_gps_location=SensorGps()
        self.angle_rad=0
        self.drone_angle=0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)


    def degrees_to_radians(self,angle_degrees:float):
        angle_radians = math.radians(angle_degrees)
        # Ensure the result is in the range from -π to π
        angle_radians = (angle_radians + math.pi) % (2*math.pi) - math.pi
        return angle_radians
    
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

    def calculate_distance_and_angle(self,utm1,utm2):
        # UTM coordinates are in the format (easting, northing, zone number, zone letter)
        easting1, northing1, _, _ = utm1
        easting2, northing2, _, _ = utm2

        # Calculate distance using Pythagorean theorem
        distance = math.sqrt((easting2 - easting1)**2 + (northing2 - northing1)**2)

        # Calculate angle using arctan2
        angle = math.atan2(northing2 - northing1, easting2 - easting1)

        return distance, angle
    
    # def vehicle_odometry_callback(self,msg):
    #     orientation=Quaternion()
    #     orientation.x=float(msg.q[0])
    #     orientation.y=float(msg.q[1])
    #     orientation.z=float(msg.q[2])
    #     orientation.w=float(msg.q[3])
    #     _,_,self.drone_angle=self.euler_from_quaternion(orientation)


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status


    def vehicle_gps_callback(self,vehicle_gps):
        self.vehicle_gps_location=vehicle_gps

    # def vehicle_imu_callback(self,msg):
    #     self.drone_angle=float(msg.delta_angle[2])

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
        msg.body_rate = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def convert_utm(self,lat:float,lon:float):
        utm_coordinate1 = utm.from_latlon(self.vehicle_gps_location.latitude_deg, self.vehicle_gps_location.longitude_deg)
        utm_coordinate2 = utm.from_latlon(lat, lon)
        distance,angle=self.calculate_distance_and_angle(utm_coordinate1,utm_coordinate2)
        return distance,angle


    def navigate_to_point(self,lat:float,lon:float,distance:float,angle:float):

            


        print(distance)
        print(angle)
        angle_rad=angle
        
        target_pos_x=self.vehicle_local_position.x+distance*math.sin(angle_rad)
        target_pos_y=self.vehicle_local_position.y+distance*math.cos(angle_rad)
        

        if self.angle_rad>=0:
            target_angle=1.57+abs(angle_rad)
        else:
            target_angle=1.57-abs(angle_rad)

        print(f"Distance: {distance:.2f} meters")
        
            
                

        # while abs(abs(self.vehicle_local_position.x)-abs(target_pos_x))>0.1:
        #     self.publish_position_setpoint(target_pos_x,self.vehicle_local_position.y,self.vehicle_local_position.z)
        #     print(abs(abs(self.vehicle_local_position.x)-abs(target_pos_x)))
        #     print(target_pos_x)
        #     print(self.vehicle_local_position.x)
            
        # while abs(abs(self.vehicle_local_position.y)-abs(target_pos_y))>0.1:
        #     self.publish_position_setpoint(self.vehicle_local_position.x,target_pos_y,self.vehicle_local_position.z)
        #     print(abs(abs(self.vehicle_local_position.y)-abs(target_pos_y)))
        #     print(target_pos_y)
        #     #     time.sleep(0.5)
        #     # while self.vehicle_local_position.y<=target_pos_y-0.1:
        #     #     self.publish_position_setpoint(self.vehicle_local_position.x,target_pos_y,self.vehicle_local_position.z,0.0)
        #     #     time.sleep(0.5)
            
        # return
        start_time = self.get_clock().now()
        timeout = Duration(seconds=5.0)  # Timeout of 2 seconds

        while self.get_clock().now() - start_time < timeout:
            self.publish_position_setpoint(target_pos_x,target_pos_y,self.takeoff_height,yaw_angle=self.drone_angle)

        # start_time = self.get_clock().now()
        # timeout = Duration(seconds=5.0) 

        # while self.get_clock().now() - start_time < timeout:
        #     self.publish_position_setpoint(self.vehicle_local_position.x,self.vehicle_local_position.y,self.takeoff_height,1.57)
            
            # do your stuff



        
            

        # self.publish_vehicle_command(19,param1=7.0,param3=0.1,param4=1.57,param5=lat,param6=lon,param7=self.takeoff_height)
        # for i in range(10000):
        #     self.publish_position_setpoint(self.vehicle_local_position.x,target_pos_y,self.takeoff_height)
            

        return True


        


    def publish_attitude_setpoint(self,yaw_angle:float):
        msg=VehicleAttitudeSetpoint()
        msg.roll_body=0.0
        msg.pitch_body=0.0
        msg.yaw_body=yaw_angle
        msg.yaw_sp_move_rate=0.05
        while abs(self.drone_angle-yaw_angle)>0.1:

            self.vehicle_attitude_publisher.publish(msg)


        

    def sort_waypoints(self,waypoints:list):

        while waypoints:
            
            
            distance=10000
            angle=10000
            target_lat=10000
            target_lon=10000
            idx=-1
            for i,w in enumerate(waypoints):
                
                dist,ang=self.convert_utm(w[0],w[1])

                if dist<distance:
                    idx=i
                    target_lat=w[0]
                    target_lon=w[1]
                    distance=dist
                    angle=ang
            
            
            print(f"{target_lat},{target_lon}")
            self.navigate_to_point(target_lat,target_lon,distance,angle)
            
            waypoints.pop(idx)

                


        


    def publish_position_setpoint(self, x: float, y: float, z: float,yaw_angle:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        # msg.velocity=[0.1,0.0,0.0]
        msg.yaw=yaw_angle
        msg.yawspeed=0.1

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")


    def publish_velocity_setpoint(self,x:float):

        msg=TrajectorySetpoint()
        msg.velocity=[x,0,0]
        msg.timestamp=int(self.get_clock().now().nanoseconds/1000)
        msg.yaw=self.angle_rad
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity setpoints {[x, 0, 0]}")

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

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height-0.5,1.57)

        elif self.vehicle_local_position.z <= self.takeoff_height:
            lat=47.39779371134151
            lon=8.545604635710545
            distance,angle=self.convert_utm(lat,lon)
            # print(distance,angle)
            # waypoints=[(47.39779806134497,8.545671128868838),(47.39779371134151, 8.545604635710545)]
            # self.navigate_to_point(47.39779806134497,8.545671128868838,distance,angle)
            # self.sort_waypoints(waypoints)
            # exit(0)
            if(self.navigate_to_point(lat,lon,distance,angle)):
                self.get_logger().info('Finished reaching')
                self.land()




        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


# class navigating_point(Node):
#     def __init__(self) -> None:
#         super().__init__('navigation task')





def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()    
    rclpy.spin(offboard_control)
    # offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)


# # - alt: 493.1202087402344
# #   latitude: 47.39774927675293
# #   longitude: 8.545609936196678
# # - alt: 493.11126708984375
# #   latitude: 47.39779371134151
# #   longitude: 8.545604635710545
# # - alt: 493.1399230957031
# #   latitude: 47.39779806134497
# #   longitude: 8.545671128868838