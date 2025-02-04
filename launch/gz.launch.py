import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    world_file_name = 'empty.world'
    world = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', package_name,
                                   'z', '0.1'],
                        output='screen',
                        parameters=[{'use_sim_time': True}]
                        # namespace=package_name
                    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="diff_drive_controller",
        arguments=["diff_drive_controller"],
        parameters=[{'use_sim_time': True}]
        # namespace=package_name
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}]
        # namespace=package_name
    )

    default_world = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','controllers.yaml')
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controller_params_file,
            {"use_sim_time": True},
            {"tf_buffer_duration": 10.0}],
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
        world_arg,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        slam,
        amcl, 
        navigation,
        twist_mux,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
