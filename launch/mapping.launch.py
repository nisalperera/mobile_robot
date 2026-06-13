"""mapping.launch.py

Launches the robot in MAPPING mode using SLAM Toolbox (online async).
AMCL and Nav2 are NOT started — use localization_nav.launch.py for those.

Launch arguments
----------------
sim_mode        : true (default) | false
    true  → starts Gazebo + bridges. Set false for the real Jetson robot.
use_sim_time    : true (default) | false
use_ros2_control: true (default) | false
headless        : true (default) | false
    true  → does not launch RViz.
world           : world name or absolute path (default: ignition_world)
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

    # ── Launch arguments ────────────────────────────────────────────────────
    sim_mode         = LaunchConfiguration('sim_mode')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    headless         = LaunchConfiguration('headless')
    world            = LaunchConfiguration('world')

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

    # ── Gazebo (sim mode only) ───────────────────────────────────────────────
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

    # ── ros2_control + spawners (real-robot only) ───────────────────────────
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

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller',
        condition=UnlessCondition(sim_mode),
        arguments=['diff_drive_controller'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster',
        condition=UnlessCondition(sim_mode),
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Joystick / teleop ────────────────────────────────────────────────────
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── Twist mux ────────────────────────────────────────────────────────────
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    # ── SLAM Toolbox (online async — mapping mode) ───────────────────────────
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

    # ── RViz (optional, skip if headless) ────────────────────────────────────
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(headless),
    )

    return LaunchDescription([
        # ── Declare args ────────────────────────────────────────────────────
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
            default_value='ignition_world',
            description=(
                'World to load in Gazebo. Accepts a name from worlds/ or an '
                'absolute path. e.g. world:=house_world'
            ),
        ),
        LogInfo(msg='[mapping.launch.py] Mode: MAPPING — SLAM Toolbox active, AMCL/Nav2 NOT started.'),
        # ── RSP (all modes) ─────────────────────────────────────────────────
        robot_state_publisher,
        # ── Sim OR real-robot hardware stack ────────────────────────────────
        gazebo_sim,
        control_node,
        diff_drive_spawner,
        joint_broad_spawner,
        # ── Common nodes ────────────────────────────────────────────────────
        joystick,
        twist_mux,
        slam,
        rviz,
    ])
