import os

from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    
    package_name = 'mobile_robot'
    
    xacro_file = get_package_share_directory(package_name) + '/description/robot.urdf.xacro'

    # Force to use sim time
    sim_time = LaunchDescription([
        SetParameter(name='use_sim_time', value=True)
    ])

	# RViz
    # if os.path.exists(get_package_share_directory(package_name) + "/config/camera.rviz"):
    #     rviz_config_file = get_package_share_directory(package_name) + "/config/camera.rviz"
    # else:
    rviz_config_file = get_package_share_directory(package_name) + "/config/default.rviz"
    rviz_node = Node(package='rviz2',
					 executable='rviz2',
					 name='rviz2',
					 output='log',
					 arguments=['-d', rviz_config_file]
                )

	# Robot State Publisher 
    robot_state_publisher = Node(package='robot_state_publisher',
								 executable='robot_state_publisher',
								 name='robot_state_publisher',
								 output='both',
								 parameters=[{'robot_description': Command(['xacro', ' ', xacro_file])           
								}])


	# Joint State Publisher 
    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
									executable='joint_state_publisher_gui',
									output='screen',
									name='joint_state_publisher_gui')


    return LaunchDescription([
        sim_time,
        robot_state_publisher, 
        joint_state_publisher_gui, 
        rviz_node,
    ])
