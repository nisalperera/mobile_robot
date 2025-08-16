import os
import logging

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, SetParameter


# logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('launch')

def log_args(context, *args, **kwargs):
    package_name = 'mobile_robot'
    logger.info(
        f"Launching {package_name} with use_sim_time: {LaunchConfiguration('use_sim_time').perform(context)}, "
        f"use_ros2_control: {LaunchConfiguration('use_ros2_control').perform(context)}, "
        f"use_slam: {LaunchConfiguration('use_slam').perform(context)}, "
    )

def generate_launch_description():

    package_name='mobile_robot'

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory(package_name),'launch','rviz.launch.py')]), 
                    launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items(),
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    world_file_name = 'obstacles.world'
    world = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file, "world": world}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', package_name],
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time}]
                    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Use NAV2 toolkit if true'),
        OpaqueFunction(function=log_args),
        rviz,
        gazebo,
        spawn_entity,
    ])
