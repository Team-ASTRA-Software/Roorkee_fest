# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus,FollowTarget


# class OffboardControl(Node):
#     """Node for controlling a vehicle in offboard mode."""

#     def __init__(self) -> None:
#         super().__init__('offboard_control_takeoff_and_land')

#         # Configure QoS profile for publishing and subscribing
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         # Create publishers
#         self.offboard_control_mode_publisher = self.create_publisher(
#             OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
#         self.trajectory_setpoint_publisher = self.create_publisher(
#             TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
#         self.vehicle_command_publisher = self.create_publisher(
#             VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
#         self.follow_target_pub=self.create_publisher(
#             FollowTarget, '/fmu/in/follow_target', qos_profile)

#         # Create subscribers
#         self.vehicle_local_position_subscriber = self.create_subscription(
#             VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
#         self.vehicle_status_subscriber = self.create_subscription(
#             VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

#         # Initialize variables
#         self.offboard_setpoint_counter = 0
#         self.vehicle_local_position = VehicleLocalPosition()
#         self.vehicle_status = VehicleStatus()
#         self.takeoff_height = -5.0
#         self.follow_target=FollowTarget()

#         # Create a timer to publish control commands
#         self.timer = self.create_timer(0.1, self.timer_callback)

#     def vehicle_local_position_callback(self, vehicle_local_position):
#         """Callback function for vehicle_local_position topic subscriber."""
#         self.vehicle_local_position = vehicle_local_position

#     def publish_target(self,x:float,y:float,z:float):
#         self.follow_target.lat=x
#         self.follow_target.lon=y
#         self.follow_target.alt=z

#         self.follow_target_pub.publish(self.follow_target)
        
        


#     def vehicle_status_callback(self, vehicle_status):
#         """Callback function for vehicle_status topic subscriber."""
#         self.vehicle_status = vehicle_status

#     def arm(self):
#         """Send an arm command to the vehicle."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
#         self.get_logger().info('Arm command sent')

#     def disarm(self):
#         """Send a disarm command to the vehicle."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
#         self.get_logger().info('Disarm command sent')

#     def engage_offboard_mode(self):
#         """Switch to offboard mode."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
#         self.get_logger().info("Switching to offboard mode")

#     def land(self):
#         """Switch to land mode."""
#         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
#         self.get_logger().info("Switching to land mode")

#     def publish_offboard_control_heartbeat_signal(self):
#         """Publish the offboard control mode."""
#         msg = OffboardControlMode()
#         msg.position = True
#         msg.velocity = False
#         msg.acceleration = False
#         msg.attitude = False
#         msg.body_rate = False
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.offboard_control_mode_publisher.publish(msg)

#     def publish_position_setpoint(self, x: float, y: float, z: float):
#         """Publish the trajectory setpoint."""
#         msg = TrajectorySetpoint()
#         msg.position = [x, y, z]
#         msg.yaw = 1.57079  # (90 degree)
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.trajectory_setpoint_publisher.publish(msg)
#         self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

#     def publish_vehicle_command(self, command, **params) -> None:
#         """Publish a vehicle command."""
#         msg = VehicleCommand()
#         msg.command = command
#         msg.param1 = params.get("param1", 0.0)
#         msg.param2 = params.get("param2", 0.0)
#         msg.param3 = params.get("param3", 0.0)
#         msg.param4 = params.get("param4", 0.0)
#         msg.param5 = params.get("param5", 0.0)
#         msg.param6 = params.get("param6", 0.0)
#         msg.param7 = params.get("param7", 0.0)
#         msg.target_system = 1
#         msg.target_component = 1
#         msg.source_system = 1
#         msg.source_component = 1
#         msg.from_external = True
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.vehicle_command_publisher.publish(msg)

#     def timer_callback(self) -> None:
#         """Callback function for the timer."""
#         self.publish_offboard_control_heartbeat_signal()

#         if self.offboard_setpoint_counter == 10:
#             self.engage_offboard_mode()
#             self.arm()

#         if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
#             # self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
#             # if self.vehicle_local_position.z<self.takeoff_height+0.1:
#                 self.publish_target(47.39774927675293,8.545609936196678,493.1202087402344)

#         # elif self.vehicle_local_position.z <= self.takeoff_height:
#         #     self.land()
#         #     exit(0)

#         if self.offboard_setpoint_counter < 11:
#             self.offboard_setpoint_counter += 1


# def main(args=None) -> None:
#     print('Starting offboard control node...')
#     rclpy.init(args=args)
#     offboard_control = OffboardControl()
#     rclpy.spin(offboard_control)
#     offboard_control.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     try:
#         main()
#     except Exception as e:
#         print(e)


# import rclpy
# from geometry_msgs.msg import PointStamped, TransformStamped
# from geographic_msgs.msg import GeoPointStamped
# import tf2_ros
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from tf2_ros import Buffer, TransformListener
# from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
# from px4_msgs.msg import SensorGps
# from rclpy.node import Node


# # Buffer.register_message_datatype(GeoPointStamped)



# class GpsToUtmConverter:
#     def __init__(self):
#         self.node = rclpy.create_node('gps_to_utm_converter')
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self.node)

#         self.gps_subscriber = self.node.create_subscription(
#             GeoPointStamped,
#             '/gps_topic',
#             self.gps_callback,
#             qos_profile
#         )
#         # self.gps_sub_drone=self.node.create_subscription(
#         #     SensorGps,
#         #     '/vehicle_gps_position',
#         #     self.gps_drone_callback,
#         #     10
#         # )
#         # self.gps_cordinates=GeoPointStamped()


#     def gps_callback(self, msg):
        
#         try:    
#             # Look up the UTM transformation
#             utm_transform = self.tf_buffer.lookup_transform('utm', msg.header.frame_id, self.node.get_clock().now().to_msg())

#             # Convert GPS to UTM
#             utm_point = self.transform_point(msg.position, utm_transform)

#             print(f'UTM Coordinates: x={utm_point.x}, y={utm_point.y}, z={utm_point.z}')

#         except (LookupException, ConnectivityException, ExtrapolationException) as e:
#             self.node.get_logger().error(f'TF Lookup Error: {e}')

#     def transform_point(self, point, transform):
#         # Transform the point using the provided transform
#         transformed_point = TransformStamped()
#         transformed_point.transform = transform.transform
#         transformed_point.header.stamp = self.node.get_clock().now().to_msg()

#         # Apply the transformation
#         transformed_point = self.tf_buffer.transform(point, 'utm', self.node.get_clock().now().to_msg())

#         return transformed_point.point

# def main():
#     rclpy.init()
#     converter = GpsToUtmConverter()
#     rclpy.spin(converter.node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import utm

# # Assuming lat, lon, alt are latitude, longitude, and altitude
# lat, lon, alt = 37.7749, -122.4194, 0.0

# # Convert to UTM coordinates
# utm_coords = utm.from_latlon(lat, lon)

# # The UTM coordinates are available in utm_coords[0] (easting), utm_coords[1] (northing), utm_coords[2] (zone), utm_coords[3] (zone letter)
# x, y = utm_coords[0], utm_coords[1]

# from geometry_msgs.msg import PointStamped
# import rclpy
# import tf2_ros

# class GpsToLocalConverter:
#     def __init__(self):
#         self.node = rclpy.create_node('gps_to_local_converter')

#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

#     def convert_gps_to_local(self, x, y, z):
#         # Create a PointStamped message
#         point_stamped = PointStamped()
#         point_stamped.header.frame_id = 'utm'  # Set the frame_id to the UTM frame
#         point_stamped.point.x = x
#         point_stamped.point.y = y
#         point_stamped.point.z = z

#         try:
#             # Look up the local transformation
#             local_transform = self.tf_buffer.lookup_transform('utm', 'base_link', self.node.get_clock().now().to_msg())

#             # Transform the point to the local frame
#             local_point = self.tf_buffer.transform(point_stamped, 'utm', self.node.get_clock().now().to_msg())

#             print(f'Local Coordinates: x={local_point.point.x}, y={local_point.point.y}, z={local_point.point.z}')

#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#             self.node.get_logger().error(f'TF Lookup Error: {e}')

# def main():
#     rclpy.init()
#     converter = GpsToLocalConverter()

#     # Assuming x, y, z are the Cartesian coordinates
#     x, y, z = 123.45, 678.90, 0.0
#     converter.convert_gps_to_local(x, y, z)

#     rclpy.spin(converter.node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PointStamped, TransformStamped
# import tf2_ros

# class GpsToLocalConverter(Node):
#     def __init__(self):
#         super().__init__('gps_to_local_converter')

#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # Publish a static transform between 'local_frame' and 'utm'
#         static_transform_stamped = TransformStamped()
#         static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
#         static_transform_stamped.header.frame_id = 'utm'
#         static_transform_stamped.child_frame_id = 'local_frame'
#         static_transform_stamped.transform.translation.x = 0.0  # Adjust these values as needed
#         static_transform_stamped.transform.translation.y = 0.0
#         static_transform_stamped.transform.translation.z = 0.0
#         static_transform_stamped.transform.rotation.w = 1.0
#         self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
#         self.tf_broadcaster.sendTransform([static_transform_stamped])

#         # Create a PointStamped message
#         point_stamped = PointStamped()
#         point_stamped.header.frame_id = 'utm'  # Set the frame_id to the UTM frame
#         point_stamped.point.x = 123.45  # Replace with your desired Cartesian coordinates
#         point_stamped.point.y = 678.90
#         point_stamped.point.z = 0.0

#         # Convert Cartesian coordinates to the local frame
#         try:
#             local_point = self.tf_buffer.transform(point_stamped, 'local_frame', rclpy.time.Time())

#             print(f'Local Coordinates: x={local_point.point.x}, y={local_point.point.y}, z={local_point.point.z}')

#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#             self.get_logger().error(f'TF Lookup Error: {e}')

# def main():
#     rclpy.init()
#     converter = GpsToLocalConverter()
#     rclpy.spin(converter)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PointStamped, TransformStamped
# import tf2_ros

# class GpsToLocalConverter(Node):
#     def __init__(self):
#         super().__init__('gps_to_local_converter')

#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # Publish a static transform between 'local_frame' and 'utm'
#         static_transform_stamped = TransformStamped()
#         static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
#         static_transform_stamped.header.frame_id = 'utm'
#         static_transform_stamped.child_frame_id = 'local_frame'
#         static_transform_stamped.transform.translation.x = 0.0  # Adjust these values as needed
#         static_transform_stamped.transform.translation.y = 0.0
#         static_transform_stamped.transform.translation.z = 0.0
#         static_transform_stamped.transform.rotation.w = 1.0
#         self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
#         self.tf_broadcaster.sendTransform([static_transform_stamped])

#         self.create_timer(0.2,self.run)

#     def run(self):
#         # Create a PointStamped message
#         point_stamped = PointStamped()
#         point_stamped.header.frame_id = 'utm'  # Set the frame_id to the UTM frame
#         point_stamped.point.x = 123.45  # Replace with your desired Cartesian coordinates
#         point_stamped.point.y = 678.90
#         point_stamped.point.z = 0.0

#         # Convert Cartesian coordinates to the local frame
#         try:
#             local_point = self.tf_buffer.transform(point_stamped, 'local_frame', self.get_clock().now().to_msg())

#             print(f'Local Coordinates: x={local_point.point.x}, y={local_point.point.y}, z={local_point.point.z}')

#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#             self.get_logger().error(f'TF Lookup Error: {e}')

# def main():
#     rclpy.init()
#     converter = GpsToLocalConverter()    
#     rclpy.spin(converter)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped,Point
import tf2_ros
from tf2_ros import TransformException, ConnectivityException, ExtrapolationException
import math

class GpsToLocalConverter(Node):
    def __init__(self):
        super().__init__('gps_to_local_converter')

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publish a static transform between 'local_frame' and 'utm'
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'utm'
        static_transform_stamped.child_frame_id = 'local_frame'
        static_transform_stamped.transform.translation.x = 0.0  # Adjust these values as needed
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform([static_transform_stamped])

        self.create_timer(0.2, self.run)

    def run(self):
        # Create a PointStamped message
        point_stamped = PointStamped()
        point_stamped.header.frame_id = 'utm'  # Set the frame_id to the UTM frame
        point_stamped.point.x = 123.45  # Replace with your desired Cartesian coordinates
        point_stamped.point.y = 678.90
        point_stamped.point.z = 0.0

        try:
            # Manually calculate the transformed coordinates
            local_point = self.transform_coordinates(point_stamped.point)

            print(f'Local Coordinates: x={local_point.x}, y={local_point.y}, z={local_point.z}')

        except (TransformException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'TF Lookup Error: {e}')

    def transform_coordinates(self, point):
        # Get the static transform parameters
        translation = [0.0, 0.0, 0.0]  # Adjust these values as needed
        rotation = [0.0, 0.0, 0.0, 1.0]  # Quaternion [x, y, z, w]

        # Apply the translation
        transformed_point = Point()
        transformed_point.x = point.x - translation[0]
        transformed_point.y = point.y - translation[1]
        transformed_point.z = point.z - translation[2]

        # Apply the rotation
        rotated_point = self.rotate_point(transformed_point, rotation)

        return rotated_point

    def rotate_point(self, point, quaternion):
        # Convert quaternion to rotation matrix
        rotation_matrix = self.quaternion_to_rotation_matrix(quaternion)

        # Apply rotation matrix to the point
        rotated_point = [
            rotation_matrix[0][0] * point[0] + rotation_matrix[0][1] * point[1] + rotation_matrix[0][2] * point[2],
            rotation_matrix[1][0] * point[0] + rotation_matrix[1][1] * point[1] + rotation_matrix[1][2] * point[2],
            rotation_matrix[2][0] * point[0] + rotation_matrix[2][1] * point[1] + rotation_matrix[2][2] * point[2]
        ]

        return rotated_point

    def quaternion_to_rotation_matrix(self, quaternion):
        x, y, z, w = quaternion

        # Normalize quaternion
        length = math.sqrt(x**2 + y**2 + z**2 + w**2)
        x /= length
        y /= length
        z /= length
        w /= length

        # Convert quaternion to rotation matrix
        rotation_matrix = [
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ]

        return rotation_matrix

def main():
    rclpy.init()
    converter = GpsToLocalConverter()
    rclpy.spin(converter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

