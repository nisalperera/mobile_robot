"""gz.launch.py

Ignition Gazebo simulation launcher.

Ignition Fortress sensor frame_id namespacing issue:
  All sensors get header.frame_id set to their fully-namespaced internal
  path, e.g.:
    lidar  -> 'mobile_robot/base_footprint/laser'
    cameras -> 'mobile_robot/base_footprint/left_camera'
                'mobile_robot/base_footprint/right_camera'

  The ROS TF tree only knows the URDF link names:
    'laser_frame', 'left_camera_link_optical', 'right_camera_link_optical'

  RViz and SLAM Toolbox drop every message because the frame_id never
  resolves in TF.

Fix: topic_tools transform nodes rewrite frame_id on each sensor topic
before any ROS node sees it:
  /scan          -> /scan_fixed          (frame_id: laser_frame)
  /left_camera/image  -> /left_camera/image_fixed  (frame_id: left_camera_link_optical)
  /right_camera/image -> /right_camera/image_fixed (frame_id: right_camera_link_optical)
"""

import logging
import os

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

    # =========================================================================
    # Frame-id fixers
    #
    # Ignition Fortress sets header.frame_id to the fully-namespaced internal
    # sensor path for every sensor. None of these exist in the ROS TF tree.
    # Each fixer subscribes the raw bridged topic, rewrites frame_id to the
    # correct URDF link name, and republishes on a _fixed topic.
    #
    # Downstream consumers (RViz, SLAM Toolbox, perception nodes) must
    # subscribe the _fixed topics, NOT the raw bridge topics.
    # =========================================================================

    # --- Lidar ---------------------------------------------------------------
    # Raw frame_id:  'mobile_robot/base_footprint/laser'
    # Fixed frame_id: 'laser_frame'
    scan_frame_fixer = Node(
        package='topic_tools',
        executable='transform',
        name='scan_frame_fixer',
        arguments=[
            '/scan',
            '/scan_fixed',
            'sensor_msgs/msg/LaserScan',
            "sensor_msgs.msg.LaserScan("
            "header=std_msgs.msg.Header("
            "stamp=m.header.stamp, "
            "frame_id='laser_frame'), "
            "angle_min=m.angle_min, "
            "angle_max=m.angle_max, "
            "angle_increment=m.angle_increment, "
            "time_increment=m.time_increment, "
            "scan_time=m.scan_time, "
            "range_min=m.range_min, "
            "range_max=m.range_max, "
            "ranges=m.ranges, "
            "intensities=m.intensities)",
            '--import', 'sensor_msgs', 'std_msgs',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Left camera ---------------------------------------------------------
    # Raw frame_id:  'mobile_robot/base_footprint/left_camera'
    # Fixed frame_id: 'left_camera_link_optical'
    # Consumers: RViz Image display, any vision pipeline
    left_camera_frame_fixer = Node(
        package='topic_tools',
        executable='transform',
        name='left_camera_frame_fixer',
        arguments=[
            '/left_camera/image',
            '/left_camera/image_fixed',
            'sensor_msgs/msg/Image',
            "sensor_msgs.msg.Image("
            "header=std_msgs.msg.Header("
            "stamp=m.header.stamp, "
            "frame_id='left_camera_link_optical'), "
            "height=m.height, "
            "width=m.width, "
            "encoding=m.encoding, "
            "is_bigendian=m.is_bigendian, "
            "step=m.step, "
            "data=m.data)",
            '--import', 'sensor_msgs', 'std_msgs',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Right camera --------------------------------------------------------
    # Raw frame_id:  'mobile_robot/base_footprint/right_camera'
    # Fixed frame_id: 'right_camera_link_optical'
    # Consumers: RViz Image display, any vision pipeline
    right_camera_frame_fixer = Node(
        package='topic_tools',
        executable='transform',
        name='right_camera_frame_fixer',
        arguments=[
            '/right_camera/image',
            '/right_camera/image_fixed',
            'sensor_msgs/msg/Image',
            "sensor_msgs.msg.Image("
            "header=std_msgs.msg.Header("
            "stamp=m.header.stamp, "
            "frame_id='right_camera_link_optical'), "
            "height=m.height, "
            "width=m.width, "
            "encoding=m.encoding, "
            "is_bigendian=m.is_bigendian, "
            "step=m.step, "
            "data=m.data)",
            '--import', 'sensor_msgs', 'std_msgs',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

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
        DeclareLaunchArgument('use_slam',          default_value='false'),
        DeclareLaunchArgument('headless',          default_value='true'),
        OpaqueFunction(function=log_args),
        OpaqueFunction(function=launch_gazebo),
        spawn_entity,
        bridge,
        scan_frame_fixer,
        left_camera_frame_fixer,
        right_camera_frame_fixer,
        delayed_spawners,
    ])
