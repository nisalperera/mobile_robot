"""gz.launch.py

Ignition Gazebo hardware launcher — simulation only.

Responsibilities:
  1. Start Ignition Gazebo (headless or with GUI)
  2. Spawn the robot entity from /robot_description
  3. Bridge Ignition ↔ ROS2 topics
  4. Spawn joint_state_broadcaster + diff_drive_controller via TimerAction

NOT responsible for:
  - robot_state_publisher  (owned by the top-level launch file)
  - RViz                   (owned by the top-level launch file)
  - SLAM / AMCL / Nav2     (owned by the top-level launch file)

This clean separation prevents duplicate RSP nodes that corrupt /tf and
/robot_description when gz.launch.py is included by mapping.launch.py or
localization_nav.launch.py.
"""

import os
import logging

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
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

    existing_plugin_path = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    existing_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    plugin_paths = ':'.join(filter(None, [
        '/opt/ros/humble/lib',
        '/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins',
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

    use_sim_time     = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    pkg_share = get_package_share_directory(package_name)

    # -------------------------------------------------------------------------
    # Spawn robot entity.
    # /robot_description must already be published by the time this runs.
    # The top-level launch file (mapping / localization_nav) owns RSP and
    # starts it before including gz.launch.py.
    # -------------------------------------------------------------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  package_name,
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # -------------------------------------------------------------------------
    # Topic bridges: Ignition ↔ ROS2
    #
    # Bridge notation:
    #   topic@ros_type@ign_type  → bidirectional
    #   topic@ros_type[ign_type  → Ignition → ROS only
    #   topic@ros_type]ign_type  → ROS → Ignition only
    #
    # /tf MUST be unidirectional (GZ→ROS only, using '[').
    # If made bidirectional ('@@'), a TF loop forms:
    #   Gazebo publishes odom→base_footprint on /tf
    #   → bridged into ROS
    #   → RSP publishes base_footprint→base_link on /tf
    #   → bridged BACK into Gazebo
    #   → Gazebo re-emits a conflicting base_footprint transform
    #   → RViz TF tree breaks: all links lose transform to base_footprint
    #
    # The correct TF chain is:
    #   odom → base_footprint   (published by diff_drive_controller via GZ /tf bridge)
    #   base_footprint → base_link  (published by robot_state_publisher, fixed joint)
    #   base_link → chassis/wheels/sensors  (published by robot_state_publisher)
    # -------------------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            f'/model/{package_name}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            f'/model/{package_name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # GZ→ROS ONLY: prevents the TF loop that breaks RViz transforms.
            # Gazebo's diff_drive_controller publishes odom→base_footprint here.
            # RSP's joint transforms (base_footprint→base_link etc.) are published
            # directly on the ROS /tf topic and do NOT need to go back into Gazebo.
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

    # -------------------------------------------------------------------------
    # Controller spawners.
    # TimerAction(5 s) gives Ignition + gz_ros2_control time to fully
    # initialise before spawner connects to /controller_manager.
    # -------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delayed_controller_spawners = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
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
            description='Passed through from parent launch (unused here)'),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo server-only (no GUI). Set false to show Gazebo GUI.'),
        OpaqueFunction(function=log_args),
        OpaqueFunction(function=launch_gazebo),
        spawn_entity,
        bridge,
        delayed_controller_spawners,
    ])
