from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Flattens the 3D gpu_lidar point cloud into a proper single-ring 2D
    sensor_msgs/LaserScan published directly on /scan, for consumption by
    2D-only pipelines (SLAM Toolbox's scan_topic via scan_frame_fixer's
    /scan_fixed, Nav2 2D costmap layers) that cannot correctly interpret
    a multi-vertical-ring LaserScan.

    Ignition's gpu_lidar sensor topic was renamed 'scan' -> 'scan_2d' in
    description/xacro/lidar.xacro (this sensor now has 32 vertical rings,
    so Ignition's native LaserScan on that topic is degenerate/unusable --
    LaserScan can only carry one ring). The bridged PointCloud2 data lives
    on /scan_2d/points; this node subscribes to that and republishes a
    valid single-ring scan on ROS /scan, exactly the topic scan_frame_fixer
    (in gz.launch.py) already expects as input -- so no change is needed
    there. This node is effectively a drop-in replacement for Ignition's
    own /scan output.

    Horizontal FOV/sample count: 150 deg total (+/-75 deg), 690 samples.
    angle_increment now uses full float precision (not rounded to 6
    decimals) so the node's internal computed reading count exactly
    matches 690 -- a rounded increment previously produced 689 readings,
    which SLAM Toolbox logged as
    "LaserRangeScan contains 689 range readings, expected 690" on every
    scan (harmless but noisy).

    QoS (BUGFIX, feature/3d-lidar): topic_tools/transform (scan_frame_fixer
    in gz.launch.py) does NOT accept an explicit output QoS override --
    it auto-discovers the QoS of whatever it is subscribed to and
    republishes with the same reliability/durability. Since this node's
    /scan publisher previously had no explicit QoS (defaulting to
    Reliable), scan_frame_fixer inherited Reliable for /scan_fixed too,
    and neither RViz's LaserScan/PointCloud displays nor any other
    Best-Effort ("sensor data") subscriber could ever connect to /scan
    or /scan_fixed. qos_overrides below forces this node's /scan
    publisher to Best Effort at the source, so scan_frame_fixer's
    auto-discovery picks that up and /scan_fixed inherits it too --
    no change needed in gz.launch.py.

    Height slice: min_height/max_height select points close to the
    lidar's own scan plane (laser_frame z ~ 0), i.e. the bottom-most
    ring(s) where vertical angle ~ 0 rad, matching the "rays start at
    the LIDAR's own level" geometry configured in
    description/xacro/lidar.xacro.

    ignition_laser_frame_bridge: Ignition's gpu_lidar publishes with
    frame_id 'mobile_robot/base_footprint/laser' -- an internal
    Ignition-namespaced name that does NOT exist anywhere in the
    URDF-derived TF tree published by robot_state_publisher (which only
    knows 'laser_frame'). Without a TF link, this node's own tf2 buffer
    lookup for target_frame 'laser_frame' can never resolve the transform
    and would spin forever. This static, zero-offset publisher declares
    the two frame names equivalent at the TF level.
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
            ('cloud_in', '/scan_2d/points'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'laser_frame',
            'transform_tolerance': 0.02,
            'min_height': -0.05,
            'max_height': 0.05,
            'angle_min': -1.308997,   # -75 deg
            'angle_max': 1.308997,    # +75 deg (150 deg total, matches lidar.xacro)
            'angle_increment': 0.0037997010159651666,  # 150 deg / 690 samples, full precision
            'scan_time': 0.1,         # matches gpu_lidar update_rate (10 Hz)
            'range_min': 0.3,
            'range_max': 12.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'concurrency_level': 1,
            # BUGFIX (feature/3d-lidar): force this node's /scan publisher
            # to Best Effort so scan_frame_fixer's auto-discovered output
            # QoS (and any other downstream sensor-data subscriber, e.g.
            # RViz) can actually connect. See docstring above.
            'qos_overrides': {
                '/scan': {
                    'publisher': {
                        'reliability': 'best_effort',
                        'durability': 'volatile',
                    }
                }
            },
        }],
    )

    return LaunchDescription([
        ignition_laser_frame_bridge,
        pointcloud_to_laserscan_node,
    ])
