"""mapping.launch.py


Launches the robot in MAPPING mode using SLAM Toolbox (online async).
AMCL and Nav2 are NOT started — use localization_nav.launch.py for those.


Odometry pipeline
-----------------
  diff_drive_controller --> /odom/raw
  imu_complementary_filter (real) or Ignition IMU bridge (sim) --> /imu_fixed/data
  robot_localization EKF --> /odom  (fused, IMU-stabilised)
  EKF also publishes the odom -> base_footprint TF (publish_odom_tf: false
  in controllers.yaml so only ONE node writes this transform).


Launch arguments
----------------
sim_mode        : true (default) | false
    true  → starts Gazebo + bridges. Set false for the real Jetson robot.
use_sim_time    : true (default) | false
use_ros2_control: true (default) | false
headless        : true (default) | false
    true  → does not launch RViz.
world           : world name or absolute path (default: ignition_empty_world)
    Name is resolved to <pkg_share>/worlds/<name>.world automatically.
    e.g. world:=house_world  or  world:=/tmp/my_arena.world


Example (Laptop — simulation)
-----------------------------
    ros2 launch mobile_robot mapping.launch.py
    ros2 launch mobile_robot mapping.launch.py world:=house_world


Example (Jetson — real robot)
------------------------------
    ros2 launch mobile_robot mapping.launch.py \\
        sim_mode:=false use_sim_time:=false
"""


import os


from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():


    package_name = 'mobile_robot'
    pkg_share = get_package_share_directory(package_name)


    # ── Launch arguments ───────────────────────────────────────────────────
    sim_mode = LaunchConfiguration('sim_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')


    # ── Robot State Publisher (always runs — sim AND real robot) ───────────
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro', ' ', xacro_file,
                    ' use_ros2_control:=', use_ros2_control,
                    ' sim_mode:=', sim_mode,
                ]),
                value_type=str,
            )
        }],
    )


    # ── Gazebo (sim mode only) ────────────────────────────────────────
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gz.launch.py')
        ),
        launch_arguments={
            'use_sim_time':     use_sim_time,
            'use_ros2_control': use_ros2_control,
            'use_slam':         'false',
            'world':            world,
        }.items(),
        condition=IfCondition(sim_mode),
    )


    # ── ros2_control + spawners (real-robot only) ───────────────────────
    controller_params_file = os.path.join(pkg_share, 'config', 'controllers.yaml')


    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        condition=UnlessCondition(sim_mode),
        parameters=[
            controller_params_file,
            {'use_sim_time': use_sim_time},
            {'tf_buffer_duration': 10.0},
        ],
        output='both',
        respawn=True,
    )


    # Remap /diff_drive_controller/odom -> /odom/raw so the EKF can
    # subscribe to the raw wheel encoder odometry on a separate topic
    # from the fused /odom output that SLAM and Nav2 consume.
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller',
        condition=UnlessCondition(sim_mode),
        arguments=['diff_drive_controller'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/diff_drive_controller/odom', '/odom/raw'),
        ],
    )


    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster',
        condition=UnlessCondition(sim_mode),
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )


    # ── IMU complementary filter (real robot only) ──────────────────────
    # In sim mode the Ignition IMU bridge publishes on /imu_fixed.
    # The EKF config expects /imu/data in both modes, so in sim mode
    # we remap /imu_fixed -> /imu/data on the EKF node (see below).
    imu_filter_params = os.path.join(pkg_share, 'config', 'imu_filter.yaml')
    imu_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_complementary_filter',
        output='screen',
        condition=UnlessCondition(sim_mode),
        parameters=[imu_filter_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data',     '/imu/data'),
        ],
    )


    # ── EKF (Extended Kalman Filter) ──────────────────────────────────
    # Fuses /odom/raw (wheel encoders) + /imu/data (angular velocity)
    # and publishes the smoothed estimate on /odom + odom->base_footprint TF.
    #
    # In sim mode:  Ignition publishes the IMU on /imu (no /imu/data).
    #               We remap /imu -> /imu/data here so ekf.yaml is
    #               identical in both sim and real-robot modes.
    #
    # In real mode: imu_complementary_filter already publishes /imu/data
    #               so no remapping is needed.
    #
    # The /odom/raw topic comes from:
    #   - Real robot: diff_drive_spawner remapping above
    #   - Sim mode:   gz.launch.py bridges the Ignition odom to /odom;
    #                 we remap /odom -> /odom/raw here so the EKF sees
    #                 the raw odom on the correct topic in sim too.
    #
    # BUGFIX (map-rotation-fix): the sim node previously ALSO remapped
    # its published output ('odometry/filtered') to '/odom' -- the same
    # topic it remaps its raw INPUT from. That made the node subscribe
    # to and publish on '/odom' at the same time, creating a feedback
    # loop where the filter partially fed its own fused output back in
    # as if it were fresh wheel odometry. Small turns stayed inside
    # SLAM's scan-match tolerance and self-corrected; larger turns
    # amplified through the loop faster than scan matching could
    # compensate, producing a diverging yaw estimate that SLAM Toolbox
    # read as a disconnected pose -- i.e. a rotated, overlapping map.
    #
    # Fix: do NOT remap the fused output to '/odom' in sim mode. Leave
    # it on the ROS 2 default 'odometry/filtered' topic. Nothing in
    # this launch file subscribes to a fused '/odom' topic directly --
    # SLAM Toolbox only consumes the odom->base_footprint TF that the
    # EKF publishes via publish_tf: true in ekf.yaml, so no functionality
    # is lost by dropping this remap.
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml')


    ekf_node_sim = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(sim_mode),
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            # Gazebo bridge publishes raw wheel odom on /odom
            ('/odom/raw', '/odom'),
            # Ignition IMU bridge publishes on /imu, not /imu/data
            ('/imu/data', '/imu'),
            # NOTE: fused output ('odometry/filtered') is intentionally
            # left un-remapped -- see BUGFIX comment above. It now
            # publishes to the default 'odometry/filtered' topic instead
            # of colliding with the raw-odom input topic '/odom'.
        ],
    )


    ekf_node_real = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=UnlessCondition(sim_mode),
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            # diff_drive_spawner remaps /diff_drive_controller/odom -> /odom/raw
            ('/odom/raw', '/odom/raw'),
            # imu_complementary_filter publishes on /imu/data
            ('/imu/data', '/imu/data'),
            # EKF fused output -> /odom (safe here: input is /odom/raw,
            # a different topic name, so no feedback loop is created)
            ('odometry/filtered', '/odom'),
        ],
    )


    # ── Joystick / teleop ────────────────────────────────────────────
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )


    # ── Twist mux ─────────────────────────────────────────────────
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')],
    )


    # ── PointCloud → LaserScan (3D LIDAR support) ───────────────────────
    # Flattens the gpu_lidar's 3D /scan/points into a proper single-ring
    # 2D LaserScan on /scan_2d. scan_frame_fixer (in gz.launch.py) should
    # consume /scan_2d instead of the raw Ignition /scan, since Ignition's
    # gpu_lidar -> LaserScan bridge cannot correctly represent a lidar with
    # >1 vertical sample (see description/xacro/lidar.xacro). SLAM Toolbox's
    # scan_topic (/scan_fixed, downstream of scan_frame_fixer) then receives
    # a valid 2D scan again.
    pointcloud_to_laserscan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'pointcloud_to_laserscan.launch.py')
        ),
    )


    # ── SLAM Toolbox (online async — mapping mode) ─────────────────────
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  slam_params_file,
        }.items(),
    )


    # ── RViz (optional, skip if headless) ─────────────────────────────
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config
        }.items(),
        condition=UnlessCondition(headless),
    )


    return LaunchDescription([
        # ── Declare args ────────────────────────────────────────────
        DeclareLaunchArgument(
            'sim_mode',
            default_value='true',
            description='true = Gazebo sim (Laptop), false = real robot (Jetson)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from Gazebo',
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control hardware interface',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Set true to suppress RViz (e.g. on Jetson)',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='ignition_empty_world',
            description=(
                'World to load in Gazebo. Accepts a name from worlds/ or an '
                'absolute path. e.g. world:=house_world'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='default',
            description='RViz config name (without .rviz) from the rviz/ directory'
        ),
        LogInfo(msg=(
            '[mapping.launch.py] Mode: MAPPING — SLAM Toolbox active, '
            'EKF fusing /odom/raw + /imu/data -> /odom, AMCL/Nav2 NOT started.'
        )),
        # ── RSP (all modes) ─────────────────────────────────────────
        robot_state_publisher,
        # ── Sim OR real-robot hardware stack ────────────────────────────
        gazebo_sim,
        control_node,
        diff_drive_spawner,
        joint_broad_spawner,
        # ── IMU filter (real robot only) ──────────────────────────────
        imu_filter,
        # ── EKF — fuses odom + IMU (both modes, different remappings) ──────
        ekf_node_sim,
        ekf_node_real,
        # ── Common nodes ────────────────────────────────────────────
        joystick,
        twist_mux,
        pointcloud_to_laserscan,
        slam,
        rviz,
    ])
