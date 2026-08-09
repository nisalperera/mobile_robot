"""localization_nav.launch.py

Launches the robot in LOCALIZATION + NAVIGATION mode using AMCL + Nav2.
SLAM Toolbox is NOT started — use mapping.launch.py for mapping.
A pre-built map MUST exist at the path given by the 'map' argument.

Odometry pipeline
------------------
  diff_drive_controller --> /odom/raw
  imu_complementary_filter (real) or Ignition IMU bridge (sim) --> /imu/data
  robot_localization EKF --> /odom/filtered  (fused, IMU-stabilised)
  EKF also publishes the odom -> base_footprint TF (publish_odom_tf: false
  in controllers.yaml so only ONE node writes this transform).

Launch arguments
----------------
sim_mode        : true (default) | false
use_sim_time    : true (default) | false
use_ros2_control: true (default) | false
map             : path to map YAML file
                  (default: <package>/maps/house_world_map.yaml)
headless        : true (default) | false
    true  → does not launch RViz.
world           : world name or absolute path (default: ignition_empty_world)
    Name is resolved to <pkg_share>/worlds/<name>.world automatically.
    e.g. world:=house_world  or  world:=/tmp/my_arena.world

Example (Laptop — simulation)
-----------------------------
    ros2 launch mobile_robot localization_nav.launch.py
    ros2 launch mobile_robot localization_nav.launch.py world:=house_world

    # With a custom map:
    ros2 launch mobile_robot localization_nav.launch.py \\
        world:=house_world map:=/path/to/house_map.yaml

Example (Jetson — real robot)
------------------------------
    ros2 launch mobile_robot localization_nav.launch.py \\
        sim_mode:=false use_sim_time:=false \\
        map:=/home/jetson/maps/my_map.yaml
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

    # ── Launch arguments ────────────────────────────────────────────────
    sim_mode = LaunchConfiguration('sim_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    map_yaml = LaunchConfiguration('map')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    # ── Robot State Publisher (always runs — sim AND real robot) ─────────────
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

    # ── Gazebo (sim mode only) ───────────────────────────────────────────
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

    # ── ros2_control + spawners (real-robot only) ────────────────────────
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
    # from the fused /odom/filtered output that AMCL and Nav2 consume.
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

    # ── IMU complementary filter (real robot only) ───────────────────────
    # In sim mode the Ignition IMU bridge publishes on /imu.
    # The EKF config expects /imu/data in both modes, so in sim mode
    # we remap /imu -> /imu/data on the EKF node (see below).
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

    # ── EKF (Extended Kalman Filter) ──────────────────────────────────────
    # Fuses /odom/raw (wheel encoders) + /imu/data (angular velocity)
    # and publishes the smoothed estimate on /odom/filtered + odom->base_footprint TF.
    # AMCL then only has to correct map->odom, not odom->base_footprint,
    # and Nav2's controllers/planners consume the IMU-stabilised /odom/filtered.
    #
    # In sim mode:  Ignition publishes the IMU on /imu (no /imu/data).
    #               We remap /imu -> /imu/data here so ekf.yaml is
    #               identical in both sim and real-robot modes.
    #
    # In real mode: imu_complementary_filter already publishes /imu/data
    #               so no remapping is needed.
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
            # EKF fused output -> /odom/filtered (consumed by AMCL + Nav2)
            ('odometry/filtered', '/odom/filtered'),
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
            # EKF fused output -> /odom/filtered
            ('odometry/filtered', '/odom/filtered'),
        ],
    )

    # ── Joystick / teleop ─────────────────────────────────────────────────
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── Twist mux ───────────────────────────────────────────────────────
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    # ── AMCL (localization) ────────────────────────────────────────────────
    amcl_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  amcl_params_file,
            'map':          map_yaml,
        }.items(),
    )

    # ── Nav2 stack ─────────────────────────────────────────────────────────
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── RViz (optional, skip if headless) ───────────────────────────────────
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(headless),
    )

    return LaunchDescription([
        # ── Declare args ──────────────────────────────────────────────
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
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'house_world_map.yaml'),
            description='Full path to the pre-built map YAML file',
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
        LogInfo(msg=(
            '[localization_nav.launch.py] Mode: LOCALIZATION+NAV — AMCL + Nav2 active, '
            'EKF fusing /odom/raw + /imu/data -> /odom/filtered, SLAM NOT started.'
        )),
        # ── RSP (all modes) ──────────────────────────────────────────────
        robot_state_publisher,
        # ── Sim OR real-robot hardware stack ─────────────────────────────
        gazebo_sim,
        control_node,
        diff_drive_spawner,
        joint_broad_spawner,
        # ── IMU filter (real robot only) ─────────────────────────────────
        imu_filter,
        # ── EKF — fuses odom + IMU (both modes, different remappings) ────
        ekf_node_sim,
        ekf_node_real,
        # ── Common nodes ──────────────────────────────────────────────────
        joystick,
        twist_mux,
        # ── Localization + Navigation ─────────────────────────────────────
        amcl,
        navigation,
        rviz,
    ])
