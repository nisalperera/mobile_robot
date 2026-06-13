import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command


# def _launch_robot_description(context, *args, **kwargs):
#     package_name = 'mobile_robot'
#     use_ros2_control = LaunchConfiguration('use_ros2_control')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     launch_robot_description = LaunchConfiguration('robot_description')
#     xacro_file = get_package_share_directory(package_name) + '/description/robot.urdf.xacro'

#     nodes = []

#     if launch_robot_description.perform(context).lower() == 'true':
#         # Robot State Publisher 
#         robot_state_publisher = Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='both',
#             parameters=[{
#                 'robot_description': Command(['xacro', ' ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])           
#                 }]
#         )
#         nodes.append(robot_state_publisher)

#     return nodes

def generate_launch_description():
    
    package_name = 'mobile_robot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    launch_robot_description = LaunchConfiguration('robot_description')

    rviz_config = LaunchConfiguration('rviz_config')
    rviz_config_file = get_package_share_directory(package_name) + f"/rviz/{rviz_config}.rviz"
    rviz_node = Node(package='rviz2',
					 executable='rviz2',
					 name='rviz2',
					 output='log',
					 arguments=['-d', rviz_config_file],
                )
    
    xacro_file = get_package_share_directory(package_name) + '/description/robot.urdf.xacro'
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])           
            }],
        condition=IfCondition(launch_robot_description)
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
            # OpaqueFunction(function=_launch_robot_description),
            robot_state_publisher,
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