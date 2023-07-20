
# ROS PoseTree Package

## Overview
This package provides a ROS2 wrapper for the PoseTree library, which was written by **Benjie Holson** - https://github.com/robobenjie/posetree. It allows for efficient and easy calculations with poses and transforms in a ROS2 environment. This package is especially useful for dealing with complex robot systems where multiple transforms need to be managed and calculated.

## Key Components
This package consists of several key components:

- `ros_posetree.py`: which has `RosPoseTree` class, which fetches transforms from tf2 and converts them into a format that PoseTree understands.

- `pose_marker_manager.py`: This module includes the `PoseMarker` and `PoseMarkerManager` classes for managing visualization markers in ROS. This helps the user to visualize the poses in RVIZ.

## Example

There are 2 python programs that need to be run:

- `test/tf2_publisher.py`: This script defines a `TF2Broadcaster` node in ROS2. It creates two transforms and broadcasts these transforms at regular intervals. This is used to simulate a simple `Robot` in ROS2

- `test/pose_tree_example.py`: This script demonstrates the usage of the PoseTree library in a ROS2 node. It creates various poses, adds markers for these poses, and calculates some properties relative to these poses.

## Getting Started

Install posetree
```
pip install posetree
```

compile this package with `colcon`

To run the example script, use the following command in 2 terminals:
```
python test/pose_tree_example.py
```

```
python test/tf2_publisher.py
```

Then start `rviz2` and add **TF** + **MarkerArray** to view the poses.

## License
This project is licensed under the BSD-3-Clause License.
