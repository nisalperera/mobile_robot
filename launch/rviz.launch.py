import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():

    package_name = 'mobile_robot'

    use_sim_time     = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    rviz_config      = LaunchConfiguration('rviz_config')

    # -------------------------------------------------------------------------
    # RViz config path
    #
    # WRONG (old code):
    #   rviz_config_file = get_package_share_directory(package_name)
    #                      + f"/rviz/{rviz_config}.rviz"
    #
    #   get_package_share_directory() returns a plain Python string at
    #   parse-time. LaunchConfiguration('rviz_config') is a substitution
    #   object — it only resolves to an actual string at launch-time.
    #   Python f-string concatenation calls str() on the substitution
    #   object, producing something like:
    #       "/install/mobile_robot/share/mobile_robot/rviz/
    #        <launch.substitutions.launch_configuration.LaunchConfiguration
    #        object at 0x7f...>.rviz"
    #   RViz receives that nonsense path and silently loads no config.
    #
    # CORRECT (fixed):
    #   PathJoinSubstitution defers all path construction until launch-time
    #   when every LaunchConfiguration value has been resolved.
    # -------------------------------------------------------------------------
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'rviz',
        [rviz_config, '.rviz'],   # concatenates the resolved string + '.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='default',
            description='RViz config name (without .rviz) from the rviz/ directory'),
        DeclareLaunchArgument(
            'robot_description',
            default_value='true',
            description='Launch the robot_description node if true'),
        rviz_node,
    ])

    # Conditionally add YOLO visualiser when ULTRALYTICS=true in the environment
    if os.environ.get('ULTRALYTICS', 'false').lower() == 'true':
        viz_node = Node(
            package='yolo_detection',
            executable='visualizer',
            name='viz_node',
            parameters=[{
                'image_reliability': 1,
                'enable': True,
                'log_image': True,
            }],
        )
        launch_description.add_action(viz_node)

    return launch_description
