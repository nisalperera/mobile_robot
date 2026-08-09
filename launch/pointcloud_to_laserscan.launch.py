from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Flattens the 3D gpu_lidar point cloud (/scan/points) into a proper
    single-ring 2D sensor_msgs/LaserScan (/scan_2d) for consumption by
    2D-only pipelines (SLAM Toolbox's scan_topic, Nav2 2D costmap layers)
    that cannot correctly interpret a 16-vertical-ring LaserScan produced
    natively by Ignition's gpu_lidar -> LaserScan bridge on /scan.

    Height slice: min_height/max_height select points close to the
    lidar's own scan plane (laser_frame z ~ 0), i.e. the bottom-most
    ring where vertical angle ~ 0 rad, matching the "rays start at the
    LIDAR's own level" geometry configured in description/xacro/lidar.xacro.

    ignition_laser_frame_bridge: Ignition's gpu_lidar publishes /scan and
    /scan/points with frame_id 'mobile_robot/base_footprint/laser' -- an
    internal Ignition-namespaced name that does NOT exist anywhere in the
    URDF-derived TF tree published by robot_state_publisher (which only
    knows 'laser_frame'). scan_frame_fixer (in gz.launch.py) rewrites this
    at the MESSAGE level for /scan -> /scan_fixed, but there is no
    equivalent fixer for /scan/points. Without a TF link, any consumer of
    the raw /scan/points (RViz's PointCloud2 display, and this node's own
    tf2 buffer lookup for target_frame 'laser_frame') can never resolve
    the transform and will spin forever with "queue is full" drops. This
    static, zero-offset publisher declares the two frame names equivalent
    at the TF level, fixing all consumers of /scan/points at once without
    needing a second per-topic frame-rewriting node.
    """
    ignition_laser_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ignition_laser_frame_bridge',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'laser_frame', 'mobile_robot/base_footprint/laser',
        ],
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/scan/points'),
            ('scan', '/scan_2d'),
        ],
        parameters=[{
            'target_frame': 'laser_frame',
            'transform_tolerance': 0.02,
            'min_height': -0.05,
            'max_height': 0.05,
            'angle_min': -1.500983,   # -86 deg
            'angle_max': 1.500983,    # +86 deg (172 deg total, matches lidar.xacro)
            'angle_increment': 0.008726645,  # 172 deg / 345 samples
            'scan_time': 0.1,         # matches gpu_lidar update_rate (10 Hz)
            'range_min': 0.3,
            'range_max': 12.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'concurrency_level': 1,
        }],
    )

    return LaunchDescription([
        ignition_laser_frame_bridge,
        pointcloud_to_laserscan_node,
    ])
