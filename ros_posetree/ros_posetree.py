import numpy as np
from typing import Optional, List
from posetree import CustomFramePoseTree, Transform, Pose

import rclpy
from rclpy.time import Time
import tf2_ros
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point

class RosPoseTree(CustomFramePoseTree):
    """
    A class to represent a tree of coordinate frames for ROS.

    This class is an extension of the PoseTree class, and integrates with the
    tf2 library in ROS to get transforms between frames.

    Attributes:
        tf_buffer: A tf2_ros.Buffer object used to get transforms from tf2.
        custom_frames: A dictionary mapping frame names to custom frames.
    """

    def __init__(self, node: Node):
        """
        Initialize a RosPoseTree with a ROS node.

        This constructor initializes the RosPoseTree instance with a reference to a ROS node. It sets up a tf2_ros.Buffer
        and a tf2_ros.TransformListener to listen for tf transforms, using the ROS node to create the TransformListener.
        It also retrieves a logger from the node and sets a flag indicating whether the tf buffer is ready for use.

        Args:
            node: A ROS node.
        """
        self.custom_frames = {}

        # The ros2 tf buffer, this contains all the tf transforms the TransformListener has received
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        self._node = node

        # Used to see if the ROS tf buffer is ready for use
        self._tf_ready = False

    def check_tf_ready(self, parent_frame: str, child_frame: str) -> bool:
        """
        This makes sure the ft buffer has the listened and received all the frames need to transform from parent_frame to child_frame

        Recommended defaults:
        parent_frame = "world" or "map"
        child_frame = "camera_link"
        """

        if self._tf_ready:
            return True

        try:
            transform = self._tf_buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            self._node.get_logger().info(
                f"Transform from {parent_frame} to {child_frame} is available."
            )
            self._tf_ready = True
            return True

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self._node.get_logger().warn(
                f"Transform from {parent_frame} to {child_frame} Not Ready: {str(e)}"
            )

        return False

    def _get_transform(
        self, parent_frame: str, child_frame: str, timestamp: Optional[Time] = None
    ) -> Transform:
        """
        Get a transform from tf2 and convert it to a posetree Transform.

        Args:
            parent_frame: The name of the parent frame.
            child_frame: The name of the child frame.
            timestamp: The time at which the transform is desired. Defaults to None, which gets the latest transform.

        Returns:
            A posetree Transform object representing the transform from the parent frame to the child frame.
        """
        if not timestamp:
            timestamp = rclpy.time.Time()

        try:
            transform = self._tf_buffer.lookup_transform(
                parent_frame, child_frame, timestamp
            )

            # Convert to posetree Transform
            t = transform.transform.translation
            q = transform.transform.rotation
            return Transform.from_position_and_quaternion(
                [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.InvalidArgumentException,
        ) as exception:
            # The transformed failed, so raise an exception
            self._node.get_logger().error(f"Transform from frame '{parent_frame}' -> '{child_frame}' failed: {str(exception)}")
            raise
