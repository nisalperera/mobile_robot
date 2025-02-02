import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node



def generate_launch_description():

    package_name='mobile_robot'

    # Include the Gazebo launch file, provided by the gazebo_ros package
    world_file_name = 'empty.world'
    world = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    # get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gazebo.launch.py')]),
                    # launch_arguments={"world": world}.items()
             )

    return LaunchDescription([
        gazebo
    ])
