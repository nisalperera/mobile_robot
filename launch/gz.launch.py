import os
import logging

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

logger = logging.getLogger('launch')


def log_args(context, *args, **kwargs):
    package_name = 'mobile_robot'
    logger.info(
        f"Launching {package_name} with "
        f"use_sim_time: {LaunchConfiguration('use_sim_time').perform(context)}, "
        f"use_ros2_control: {LaunchConfiguration('use_ros2_control').perform(context)}, "
        f"use_slam: {LaunchConfiguration('use_slam').perform(context)}, "
        f"headless: {LaunchConfiguration('headless').perform(context)}"
    )


def launch_gazebo(context, *args, **kwargs):
    package_name = 'mobile_robot'
    pkg_share = get_package_share_directory(package_name)

    world_file_name = 'ignition_world.world'
    world = os.path.join(pkg_share, 'worlds', world_file_name)

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz_args = f'-r -s --force-version 6 {world}' if headless else f'-r --force-version 6 {world}'

    # Collect existing env paths to not overwrite them
    existing_plugin_path = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    existing_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    plugin_paths = ':'.join(filter(None, [
        '/opt/ros/humble/lib',                                    # ign_ros2_control
        '/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins',        # core ign plugins
        existing_plugin_path,
    ]))

    resource_paths = ':'.join(filter(None, [
        '/usr/share/ignition/ignition-gazebo6',
        existing_resource_path,
    ]))

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo'] + gz_args.split(),
        output='screen',
        additional_env={
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': plugin_paths,
            'IGN_GAZEBO_RESOURCE_PATH': resource_paths,
        }
    )
    return [gazebo]


def generate_launch_description():
    package_name = 'mobile_robot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    pkg_share = get_package_share_directory(package_name)

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': use_ros2_control,
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', package_name,
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            f'/model/{package_name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            f'/model/{package_name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/left_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/left_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/right_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/right_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'expand_gz_topic_names': False},
        ],
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
            default_value='false',
            description='Use SLAM if true'),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo server-only (no GUI/rendering). '
                        'Set false once NVIDIA GPU passthrough is configured.'),
        OpaqueFunction(function=log_args),
        OpaqueFunction(function=launch_gazebo),
        rviz,
        spawn_entity,
        bridge,
    ])