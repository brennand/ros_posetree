import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


class TF2Broadcaster(Node):
    def __init__(self):
        super().__init__("tf2_broadcaster")

        # Create a tf2 broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Create transforms
        self.world_transform = TransformStamped()
        self.world_transform.header.frame_id = "world"
        self.world_transform.child_frame_id = "base_footprint"
        self.world_transform.transform.translation.x = 2.0
        self.world_transform.transform.translation.y = 3.0
        self.world_transform.transform.translation.z = 0.0
        self.world_transform.transform.rotation.w = 1.0  # Identity quaternion

        self.camera_link_transform = TransformStamped()
        self.camera_link_transform.header.frame_id = "base_footprint"
        self.camera_link_transform.child_frame_id = "camera_link"
        self.camera_link_transform.transform.translation.x = 0.0
        self.camera_link_transform.transform.translation.y = 0.0
        self.camera_link_transform.transform.translation.z = 2.0
        self.camera_link_transform.transform.rotation.w = 1.0  # Identity quaternion
        self.broadcaster.sendTransform(self.camera_link_transform)

        # Setup timer to publish transforms
        self.timer = self.create_timer(0.1, self.publish_transforms)

        self.get_logger().info("TF2Broadcaster Started")

    def publish_transforms(self):
        # Update timestamps
        self.world_transform.header.stamp = self.get_clock().now().to_msg()
        self.camera_link_transform.header.stamp = self.get_clock().now().to_msg()

        # Broadcast transforms
        self.broadcaster.sendTransform(self.world_transform)
        self.broadcaster.sendTransform(self.camera_link_transform)


def main():
    rclpy.init()
    node = TF2Broadcaster()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
