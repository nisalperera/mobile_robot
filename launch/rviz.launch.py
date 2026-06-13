import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    
    package_name = 'mobile_robot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    rviz_config = LaunchConfiguration('rviz_config')
    rviz_config_file = get_package_share_directory(package_name) + f"/rviz/{rviz_config}.rviz"
    rviz_node = Node(package='rviz2',
					 executable='rviz2',
					 name='rviz2',
					 output='log',
					 arguments=['-d', rviz_config_file],
                )

    launch_descriptions = LaunchDescription([
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
                description='Use given RViz config file from the rviz directory'),
            DeclareLaunchArgument(
                'robot_description',
                default_value='true',
                description='Launch the robot_description node if true'),
            rviz_node,
        ])

    if os.environ.get("ULTRALYTICS", "false") == "true":
        viz_node = Node(
                package='yolo_detection',
                executable='visualizer',
                name='viz_node',
                parameters=[{
                    'image_reliability': 1,
                    'enable': True,
                    'log_image': True
                }],
            )

        launch_descriptions.add_action(viz_node)
    
    return launch_descriptions