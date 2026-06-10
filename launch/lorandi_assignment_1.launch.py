from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

# Set at the Python level (before any process spawn) so Gazebo inherits the value.
# IGN_VERBOSITY: 0=silent, 1=errors, 2=warnings, 3=info, 4=debug
os.environ['IGN_VERBOSITY'] = '1'


def generate_launch_description():
    # 1) Launch the official simulation (ir_launch)
    ir_launch_dir = get_package_share_directory('ir_launch')
    assignment_launch_path = os.path.join(
        ir_launch_dir,
        'launch',
        'assignment_1.launch.py'
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(assignment_launch_path),
        launch_arguments={'use_rviz': 'false'}.items()
    )

    # 2) AprilTag node with the EXTERNAL camera
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        output='screen',
        remappings=[
            ('image_rect', '/rgb_camera/image'),  # <-- External camera!
            ('camera_info', '/rgb_camera/camera_info')
        ],
        parameters=[os.path.join(
            get_package_share_directory('lorandi_assignament_1'),
            'config',
            'apriltag_params.yaml'
        )],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 3) Tag detector node
    tag_detector_node = Node(
        package='lorandi_assignament_1',
        executable='tag_detector.py',
        name='tag_detector',
        output='screen'
    )

    # 4) Go to tags node
    go_to_tags_node = Node(
        package='lorandi_assignament_1',
        executable='go_to_tags.py',
        name='go_to_tags',
        output='screen'
    )
    # 5) Table detector node
    table_detector_node = Node(
        package='lorandi_assignament_1',
        executable='table_detector.py',
        name='table_detector',
        output='screen'
    )
    # 6) Table publisher node
    table_publisher_node = Node(
        package='lorandi_assignament_1',
        executable='table_publisher.py',
        name='table_publisher',
        output='screen'
    )
    return LaunchDescription([
        sim_launch,
        apriltag_node,
        tag_detector_node,
        go_to_tags_node,
        table_detector_node,
        table_publisher_node,
    ])
