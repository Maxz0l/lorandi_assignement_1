"""
lorandi_nodes.launch.py - Launches ONLY my nodes, in a clean terminal.

Goal: a readable display showing ONLY my nodes (each with its own colour, its
SUB/PUB/TF banners and its [TAG]/[GOAL]/[NAV]/[CORRIDOR]/[TOPIC] tags), without
the Gazebo / Nav2 / RViz noise. The professor's simulation is launched SEPARATELY,
in another terminal:

    Terminal 1 (the professor's simulation: Gazebo + Nav2 - minimize it):
        ros2 launch ir_launch assignment_1.launch.py

    Terminal 2 (my nodes, clean and coloured):
        ros2 launch lorandi_assignament_1 lorandi_nodes.launch.py

My nodes wait on their own for the simulation topics (/apriltag/detections, /odom,
/scan, TF), so the launch order is tolerant.

For an all-in-one launch (sim + nodes in the same terminal), use
lorandi_assignment_1.launch.py instead.
"""

import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _node(package, executable, name, **kwargs):
    """Node helper: screen output + emulated TTY (so the ANSI colours pass through)."""
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        emulate_tty=True,
        **kwargs,
    )


def generate_launch_description():
    # Clean console log format: [severity] [node] message
    clean_format = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
    )
    force_colors = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    apriltag_node = _node(
        "apriltag_ros", "apriltag_node", "apriltag",
        namespace="apriltag",
        remappings=[
            ("image_rect",  "/rgb_camera/image"),   # external camera
            ("camera_info", "/rgb_camera/camera_info"),
        ],
        parameters=[os.path.join(
            get_package_share_directory("lorandi_assignament_1"),
            "config", "apriltag_params.yaml",
        )],
        # apriltag floods sync warnings → keep only its errors.
        arguments=["--ros-args", "--log-level", "apriltag:=ERROR"],
    )

    tag_detector    = _node("lorandi_assignament_1", "tag_detector.py",    "tag_detector")
    go_to_tags      = _node("lorandi_assignament_1", "go_to_tags.py",      "go_to_tags")
    table_detector  = _node("lorandi_assignament_1", "table_detector.py",  "table_detector")
    table_publisher = _node("lorandi_assignament_1", "table_publisher.py", "table_publisher")

    return LaunchDescription([
        clean_format,
        force_colors,
        apriltag_node,
        tag_detector,
        go_to_tags,
        table_detector,
        table_publisher,
    ])
