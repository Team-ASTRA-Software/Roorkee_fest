import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
from rclpy.node import Node

def main():
    rclpy.init()
    node = rclpy.create_node('dynamic_tf_broadcaster')
    broadcaster = TransformBroadcaster(node)

    while rclpy.ok():
        # Broadcast dynamic transform between "gps_frame" and "utm"
        transform = TransformStamped()
        transform.header = Header()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = "utm"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        broadcaster.sendTransform(transform)

        rclpy.spin_once(node)

if __name__ == '__main__':
    main()
