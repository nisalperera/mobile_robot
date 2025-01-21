import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node



def generate_launch_description():

    package_name='mobile_robot'

    rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory(package_name),'launch','rviz.launch.py')]), 
                    launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items(),
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory(package_name),'launch','joystick.launch.py')]), 
                    launch_arguments={'use_sim_time': 'true'}.items()
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
                        # namespace=package_name
                    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="diff_drive_controller",
        arguments=["diff_drive_controller"],
        # namespace=package_name
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster",
        arguments=["joint_state_broadcaster"],
        # namespace=package_name
    )

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','controllers.yaml')
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controller_params_file,
            {"use_sim_time": True}],
        output="both",
        respawn=True,
        # namespace=package_name
    )

    slam_params_file = os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml')
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory(package_name),'launch','online_async_launch.py')]), 
                    launch_arguments={'use_sim_time': 'true', 'params_file': slam_params_file}.items()
    )

    amcl_params_file = os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory(package_name),'maps','map_save.yaml')
    amcl = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory(package_name),'launch','localization.launch.py')]), 
                    launch_arguments={
                        'use_sim_time': 'true', 
                        'params_file': amcl_params_file,
                        'map': map_file
                    }.items()
    )

    # Launch Nav2
    # navigation = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 [os.path.join(get_package_share_directory(package_name),'launch','navigation_launch.py')]), 
    #                 launch_arguments={
    #                     'use_sim_time': 'true',
    #                 }.items()
    # )

    navigation = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [os.path.join(get_package_share_directory(package_name),'launch','navigation.launch.py')]), 
                        launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_drive_controller/cmd_vel_unstamped')]
        )

    return LaunchDescription([
        rviz,
        joystick,
        control_node,  # Move this before gazebo
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        slam,
        amcl, 
        navigation,
        twist_mux,
    ])
