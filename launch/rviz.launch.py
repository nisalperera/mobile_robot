import os

from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    
    package_name = 'mobile_robot'
    
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    xacro_file = get_package_share_directory(package_name) + '/description/robot.urdf.xacro'

    # Force to use sim time
    sim_time = LaunchDescription([
        SetParameter(name='use_sim_time', value=use_sim_time)
    ])

    rviz_config_file = get_package_share_directory(package_name) + "/config/default.rviz"
    rviz_node = Node(package='rviz2',
					 executable='rviz2',
					 name='rviz2',
					 output='log',
					 arguments=['-d', rviz_config_file],
                     # namespace=package_name
                )

	# Robot State Publisher 
    robot_state_publisher = Node(package='robot_state_publisher',
								executable='robot_state_publisher',
								name='robot_state_publisher',
								output='both',
								parameters=[{
                                        'robot_description': Command(['xacro', ' ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])           
								    }],
                                # namespace=package_name
                            )


	# # Joint State Publisher 
    # joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
	# 								executable='joint_state_publisher_gui',
	# 								output='screen',
	# 								name='joint_state_publisher_gui')

    if os.environ.get("DEEPLEARNING", "false") == "true":
        import torch
        detector_node = Node(
                package='yolo_detection',
                executable='detector',
                name='yolo_node',
                parameters=[{
                    'model': 'yolov10m.pt',
                    'tracker': 'bytetrack.yaml',
                    'device': 'cuda' if torch.cuda.is_available() else 'cpu',
                    'enable': True,
                    'threshold': 0.5,
                    'image_reliability': 1,
                    'to_posestamped': True,
                    'to_pointcloud': True,
                    'visualize': True,
                    'stereo_vision': True
                }],
                # namespace=package_name
            )
        
        viz_node = Node(
                package='yolo_detection',
                executable='visualizer',
                name='viz_node',
                parameters=[{
                    'image_reliability': 1,
                    'enable': True,
                    'log_image': True
                }],
                # namespace=package_name
            )


        return LaunchDescription([
            sim_time,
            robot_state_publisher, 
            # joint_state_publisher_gui, 
            rviz_node,
            detector_node,
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
            sim_time,
            robot_state_publisher, 
            # joint_state_publisher_gui, 
            rviz_node,
        ])