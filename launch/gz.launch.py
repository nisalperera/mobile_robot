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

TF ownership:
  odom → base_footprint      → diff_drive_controller (ros2_control)
  base_footprint → base_link → robot_state_publisher (fixed joint)
  base_link → chassis → wheels/sensors → robot_state_publisher

  /tf is NOT bridged from Ignition. Ignition's DiffDrive system plugin
  publishes namespaced frames (mobile_robot/odom). Bridging those would
  overwrite the correct odom→base_footprint from diff_drive_controller.

  The LaserScan sensor topic IS bridged from Ignition → ROS as /scan.
  The full Ignition topic name is:
    /world/ignition_world/model/mobile_robot/link/laser_frame/sensor/laser/scan
  which is mapped to the ROS topic /scan via the bridge remapping below.
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

    # -------------------------------------------------------------------------
    # Spawn robot entity.
    # RSP must already be running (started by the top-level launch file)
    # before this action executes so /robot_description is available.
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
    # Topic bridges: Ignition → ROS2
    #
    # Bridge notation:
    #   topic@ros_type@ign_type  → bidirectional
    #   topic@ros_type[ign_type  → Ignition → ROS only
    #   topic@ros_type]ign_type  → ROS → Ignition only
    #
    # /tf is NOT bridged — see module docstring for why.
    #
    # LaserScan bridge:
    #   Ignition gpu_lidar publishes on the full model-namespaced topic:
    #     /world/ignition_world/model/mobile_robot/link/laser_frame/sensor/laser/scan
    #   We bridge this to the ROS topic /scan using the ros_gz_bridge
    #   topic remapping argument format:
    #     /ign_topic@ros_type[ign_type  with  --ros-args -r /ign_topic:=/ros_topic
    #   The cleaner approach supported by ros_gz_bridge is to specify
    #   the full IGN topic on the left and remap via the remappings list.
    # -------------------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            f'/model/{package_name}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            f'/model/{package_name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # LaserScan: bridge the full Ignition topic name, then remap to /scan
            f'/world/ignition_world/model/{package_name}/link/laser_frame/sensor/laser/scan'
            f'@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/left_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/left_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/right_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/right_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        remappings=[
            # Remap the full Ignition lidar topic to the short /scan ROS topic
            # that SLAM Toolbox and Nav2 expect.
            (
                f'/world/ignition_world/model/{package_name}/link/laser_frame/sensor/laser/scan',
                '/scan'
            ),
        ],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
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
