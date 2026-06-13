"""gz.launch.py

Ignition Gazebo hardware launcher — simulation only.

Responsibilities:
  1. Start Ignition Gazebo (headless or with GUI)
  2. Spawn the robot entity from /robot_description
  3. Bridge Ignition <-> ROS2 topics via config/ros_gz_bridge.yaml
  4. Spawn joint_state_broadcaster + diff_drive_controller via TimerAction

NOT responsible for:
  - robot_state_publisher  (owned by the top-level launch file)
  - RViz                   (owned by the top-level launch file)
  - SLAM / AMCL / Nav2     (owned by the top-level launch file)

TF ownership (authoritative sources only):
  map -> odom                : SLAM Toolbox (mapping) or AMCL (localization)
  odom -> base_footprint     : diff_drive_controller (ros2_control)
  base_footprint -> base_link: robot_state_publisher (fixed joint in URDF)
  base_link -> all children  : robot_state_publisher

  The Ignition /tf topic is NOT bridged. Ignition's DiffDrive system plugin
  publishes TF with namespaced frames (mobile_robot/odom). Bridging those
  overwrites the correct odom->base_footprint from diff_drive_controller.

Bridge config:
  All topic bridges are declared in config/ros_gz_bridge.yaml.
  The YAML approach is preferred over inline args because:
    - No shell-escaping issues with long Ignition topic names
    - Lidar remapping (/world/ignition_world/.../scan -> /scan) is clean
    - Easy to add/remove topics without touching launch code
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
        f"[gz.launch.py] "
        f"use_sim_time={LaunchConfiguration('use_sim_time').perform(context)} "
        f"use_ros2_control={LaunchConfiguration('use_ros2_control').perform(context)} "
        f"headless={LaunchConfiguration('headless').perform(context)}"
    )


def launch_gazebo(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mobile_robot')

    world_file = os.path.join(pkg_share, 'worlds', 'ignition_world.world')
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz_args = f'-r -s --force-version 6 {world_file}' if headless \
              else f'-r --force-version 6 {world_file}'

    existing_plugin_path  = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
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
            'IGN_GAZEBO_RESOURCE_PATH':      resource_paths,
        }
    )
    return [gazebo]


def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_share    = get_package_share_directory(package_name)

    use_sim_time     = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # -------------------------------------------------------------------------
    # Spawn robot into Ignition.
    # RSP must be running (started by the top-level launch file) BEFORE this
    # so that /robot_description is already published.
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
    # Topic bridge: Ignition <-> ROS2
    # Config lives in config/ros_gz_bridge.yaml — see that file for the full
    # topic list and direction rationale (especially why /tf is excluded).
    # -------------------------------------------------------------------------
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '--params-file', bridge_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # -------------------------------------------------------------------------
    # Controller spawners.
    # 5 s delay gives Ignition + ign_ros2_control time to fully initialise
    # the controller_manager before the spawner tries to connect.
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
    delayed_spawners = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, diff_drive_controller_spawner],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',      default_value='true'),
        DeclareLaunchArgument('use_ros2_control',  default_value='true'),
        DeclareLaunchArgument('use_slam',          default_value='false',
            description='Passed through from parent launch (unused here)'),
        DeclareLaunchArgument('headless',          default_value='true',
            description='true = server-only (no GUI), false = show Gazebo GUI'),
        OpaqueFunction(function=log_args),
        OpaqueFunction(function=launch_gazebo),
        spawn_entity,
        bridge,
        delayed_spawners,
    ])
