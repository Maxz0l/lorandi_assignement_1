from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

# Défini au niveau Python (avant tout spawn de process) pour que Gazebo hérite de la valeur.
# IGN_VERBOSITY : 0=silence, 1=erreurs, 2=warnings, 3=info, 4=debug
os.environ['IGN_VERBOSITY'] = '1'


def generate_launch_description():
    # 1) Lancement de la simulation officielle (ir_launch)
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

    # 2) Node AprilTag avec caméra EXTERNE
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        output='screen',
        remappings=[
            ('image_rect', '/rgb_camera/image'),  # <-- Caméra externe !
            ('camera_info', '/rgb_camera/camera_info')
        ],
        parameters=[os.path.join(
            get_package_share_directory('lorandi_assignament_1'),
            'config',
            'apriltag_params.yaml'
        )],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 3) Ton node
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
