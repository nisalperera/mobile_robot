from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    
    package_name = 'mobile_robot'
    
    xacro_file = get_package_share_directory(package_name) + '/description/robot.urdf.xacro'


	# RViz
    rviz_config_file = get_package_share_directory(package_name) + "/config/camera.rviz"
    rviz_node = Node(package    ='rviz2',
					 executable ='rviz2',
					 name       ='rviz2',
					 output     ='log',
					 arguments  =['-d', rviz_config_file])


	
	# Robot State Publisher 
    robot_state_publisher = Node(package    ='robot_state_publisher',
								 executable ='robot_state_publisher',
								 name       ='robot_state_publisher',
								 output     ='both',
								 parameters =[{'robot_description': Command(['xacro', ' ', xacro_file])           
								}])


	# Joint State Publisher 
    joint_state_publisher_gui = Node(package  ='joint_state_publisher_gui',
									executable='joint_state_publisher_gui',
									output    ='screen',
									name      ='joint_state_publisher_gui')


    return LaunchDescription([robot_state_publisher, joint_state_publisher_gui, rviz_node ])
	# return LaunchDescription([robot_state_publisher, rviz_node ])
