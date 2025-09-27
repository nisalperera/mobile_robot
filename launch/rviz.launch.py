import os

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    
    package_name = 'mobile_robot'

    rviz_config_file = get_package_share_directory(package_name) + "/rviz/3d_lidar.rviz"
    rviz_node = Node(package='rviz2',
					 executable='rviz2',
					 name='rviz2',
					 output='log',
					 arguments=['-d', rviz_config_file],
                )

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


        return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use sim time if true'),
            DeclareLaunchArgument(
                'use_ros2_control',
                default_value='true',
                description='Use ros2_control if true'),
            rviz_node,
            viz_node
        ])
    else:
        return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use sim time if true'),
            DeclareLaunchArgument(
                'use_ros2_control',
                default_value='true',
                description='Use ros2_control if true'),
            rviz_node,
        ])