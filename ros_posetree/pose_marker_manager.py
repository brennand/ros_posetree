import numpy as np
from typing import List
from posetree import Pose

from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point


class PoseMarker:
    """
    A class representing multiple markers with the same id.

    Attributes:
        id: The unique ID of the marker.
        markers: A list of Marker messages associated with the marker.
    """

    def __init__(self, id: int, markers: List[Marker]):
        self.id = id
        self.markers = markers


class PoseMarkerManager:
    """
    A class managing visualization markers in ROS.

    Attributes:
        _publisher: The publisher which publishes the MarkerArray message.
        _markers: A dictionary storing the PoseMarker objects, with their string IDs as keys.
        _marker_id_index: The index used to generate unique IDs for the markers.
    """

    def __init__(self, node: Node, topic: str = "/pose_marker"):
        """
        Initializes the PoseMarkerManager with a ROS node and a topic.
        """
        self._publisher = node.create_publisher(MarkerArray, topic, 10)
        self._markers = {}
        self._marker_id_index = 0

    def add_marker(self, pose: Pose, marker_id: str):
        """
        Adds a marker to the manager and publishes all markers.

        Args:
            pose: The pose of the marker to be added.
            marker_id: The string ID of the marker to be added.
        """
        id = self._get_id(marker_id)
        markers = self._create_frame_markers(pose, id)
        markers.append(self._create_lable_markers(pose, id, marker_id))

        self._markers[marker_id] = PoseMarker(id, markers)

        # Create a marker array to publish
        marker_array = MarkerArray()
        for pose_marker in self._markers.values():
            marker_array.markers.extend(pose_marker.markers)

        self._publisher.publish(marker_array)

    def _create_lable_markers(self, pose: Pose, id: int, marker_text: str) -> Marker:
        """
        Creates a text marker at the specified pose.

        This function creates a text marker at the position specified by `pose`. The
        text of the marker is set to `marker_text`. The marker is assigned an ID
        that is the sum of `id` and 3.

        Args:
            pose: The pose at which the marker is to be placed. The position and orientation
                of the pose are used to set the corresponding properties of the marker.
            id: The base ID for the marker. The actual ID of the marker is `id + 3`.
            marker_text: The text to be displayed on the marker.

        Returns:
            A marker of type `Marker.TEXT_VIEW_FACING` representing a text label at the specified pose.
        """
        marker = Marker()
        marker.header = Header(frame_id=pose.frame)
        marker.id = id + 3
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = marker_text
        marker.action = Marker.ADD
        marker.pose.position = Point(
            x=pose.position[0], y=pose.position[1], z=pose.position[2] - 0.05
        )
        q = pose.rotation.as_quat()
        (
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ) = q
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        return marker

    def _create_frame_markers(self, pose: Pose, id: int) -> List[Marker]:
        """
        Creates markers for a coordinate frame.

        This function creates an arrow marker for each axis of the frame,
        with the x-axis shown as red, the y-axis as green, and the z-axis as blue.
        The arrows are rotated according to the orientation of the pose.

        Args:
            pose: The pose of the frame.
            id: The id for the markers.

        Returns:
            A list of markers representing the coordinate frame.
        """

        # Define colors for axes (RGB standard)
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]

        r_orientation = pose
        g_orientation = pose.rotate_about_z(np.pi / 2)
        b_orientation = pose.rotate_about_y(-(np.pi / 2))

        orientation = [
            r_orientation.rotation.as_quat(),
            g_orientation.rotation.as_quat(),
            b_orientation.rotation.as_quat(),
        ]

        markers = []
        for i in range(3):
            marker = Marker()
            marker.header = Header(frame_id=pose.frame)
            marker.id = id + i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position = Point(
                x=pose.position[0], y=pose.position[1], z=pose.position[2]
            )

            (
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w,
            ) = orientation[i]

            marker.scale.x = 0.2
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.a = 1.0
            marker.color.r = colors[i][0]
            marker.color.g = colors[i][1]
            marker.color.b = colors[i][2]
            markers.append(marker)

        return markers

    def _get_id(self, marker_id: str) -> int:
        """
        Returns the unique integer ID of the marker with the given string ID.
        If the marker does not exist, a new ID is generated and returned.

        Args:
            marker_id: The string ID of the marker.

        Returns:
            The unique integer ID of the marker.
        """
        if marker_id not in self._markers:
            self._marker_id_index += 4
            return self._marker_id_index

        return self._markers[marker_id].id

    def delete_marker(self, marker_id: str):
        """
        Deletes a marker from the manager and from the visualization.

        Args:
            marker_id: The string ID of the marker to be deleted.
        """
        # Check that the marker_id is valid
        if not marker_id in self._markers:
            return

        # Get the markers to delete
        markers = self._markers[marker_id].markers

        # loop through all the markers and tag them for delete
        for marker in markers:           
            marker.action = Marker.DELETE

        # Marker array msg and publish
        marker_array_msg = MarkerArray()
        marker_array_msg.markers.extend(markers)   
        self._publisher.publish(marker_array_msg)

        # Clear the markers so they are not published
        self._markers[marker_id].markers = []

    def clear_all_markers(self):
        """
        Deletes all markers from the manager and from the visualization.
        """
        for marker_id in list(self._markers.keys()):
            self.delete_marker(marker_id)
