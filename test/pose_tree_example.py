import numpy as np

from scipy.spatial.transform import Rotation
from posetree import Pose

import rclpy
from rclpy.node import Node

from ros_posetree import RosPoseTree, PoseMarkerManager


class PoseTreeNode(Node):
    def __init__(self):
        super().__init__("posetree")

        self.pose_tree = RosPoseTree(node=self)
        self.pose_marker = PoseMarkerManager(node=self)

        # The control loop
        self.timer = self.create_timer(1.0, self.control_loop)

        self.get_logger().info(f"Pose Tree Started")

        self.i = 0

    def control_loop(self):
        # Check to make sure the tf transforms are ready for processing
        if not self.pose_tree.check_tf_ready("world", "camera_link"):
            return

        self.get_logger().info(f"\n\n Process Transforms \n")

        # Create a pose from perception information
        pose_of_tea_bottle = Pose.from_position_and_rotation(
            [-0.3, 0.4, 1.2],
            Rotation.identity(),
            parent_frame="camera_link",
            pose_tree=self.pose_tree,
        )
        self.pose_marker.add_marker(pose_of_tea_bottle, "pose_of_tea_bottle")

        gripper_pose = Pose.from_position_and_rotation(
            [0.2, 0.1, 0.7],
            Rotation.identity(),
            parent_frame="base_footprint",
            pose_tree=self.pose_tree,
        )
        self.pose_marker.add_marker(gripper_pose, "gripper_pose")

        base_pose = Pose.from_position_and_rotation(
            [0.0, 0.0, 0.0],
            Rotation.identity(),
            parent_frame="base_footprint",
            pose_tree=self.pose_tree,
        )
        self.pose_marker.add_marker(base_pose, "base_pose")

        human_pose = Pose.from_position_and_rotation(
            [-0.3, 0.4, 1.2],
            Rotation.identity(),
            parent_frame="world",
            pose_tree=self.pose_tree,
        )
        self.pose_marker.add_marker(human_pose, "human_pose")

        # Calculate things based on other frames
        height_of_tea = pose_of_tea_bottle.in_frame("world").z
        self.get_logger().info(f"height_of_tea: {height_of_tea}")

        distance_from_gripper = pose_of_tea_bottle.distance_to(gripper_pose)
        self.get_logger().info(f"distance_from_gripper: {distance_from_gripper}")

        # Calculate some base motion targets relative to the current robot pose
        base_target = base_pose.translate([2, 0, 0]).rotate_about_z(np.pi / 4)

        # Get numbers out of a pose to send to a motion API
        x, y, _ = base_target.in_frame("world").position
        self.get_logger().info(f"base_target: {x}, {y}")

        self.i += 1

        if self.i > 2:
            self.get_logger().info(f"\n\n Delete \n\n")
            self.pose_marker.clear_all_markers()
            self.i = 0

def main():
    rclpy.init()
    node = PoseTreeNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
