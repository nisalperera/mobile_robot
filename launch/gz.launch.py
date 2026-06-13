"""gz.launch.py

Ignition Gazebo hardware launcher — simulation only.
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

    existing_plugin_path   = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
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
    # -------------------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/mobile_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/mobile_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/left_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/left_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/right_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/right_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        remappings=[
            ('/model/mobile_robot/odometry', '/diff_drive_controller/odom'),
            ('/model/mobile_robot/cmd_vel',  '/diff_drive_controller/cmd_vel_unstamped'),
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # -------------------------------------------------------------------------
    # frame_id fixer for /scan
    #
    # Ignition Fortress gpu_lidar sets header.frame_id to its internal
    # namespaced path: 'mobile_robot/base_footprint/laser'
    # The ROS TF tree only knows 'laser_frame' (from robot_state_publisher).
    # SLAM Toolbox and RViz drop every message because the frame never
    # matches, producing the 'queue is full' error loop.
    #
    # This node rewrites frame_id to 'laser_frame' on every incoming scan.
    # SLAM Toolbox and RViz must subscribe to /scan_fixed, not /scan.
    # -------------------------------------------------------------------------
    scan_frame_fixer = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_frame_fixer',
        # Re-use parameter_bridge just to relay; frame_id is fixed below
        # via a small Python relay node instead
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Python relay: rewrites frame_id and republishes on /scan_fixed
    scan_relay = Node(
        package='mobile_robot',
        executable='scan_frame_fixer',
        name='scan_frame_fixer',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'target_frame': 'laser_frame'},
        ],
        output='screen',
    )

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
        DeclareLaunchArgument('use_slam',          default_value='false'),
        DeclareLaunchArgument('headless',          default_value='true'),
        OpaqueFunction(function=log_args),
        OpaqueFunction(function=launch_gazebo),
        spawn_entity,
        bridge,
        scan_relay,
        delayed_spawners,
    ])
