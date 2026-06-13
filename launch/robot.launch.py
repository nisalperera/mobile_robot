import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'mobile_robot'

    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    use_sim_time   = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_slam       = LaunchConfiguration('use_slam')
    # sim_mode is now an EXPLICIT argument, decoupled from use_sim_time.
    # On the real robot pass sim_mode:=false use_sim_time:=false.
    # In simulation (default) both are true.
    sim_mode       = LaunchConfiguration('sim_mode')

    pkg_share    = get_package_share_directory(package_name)
    xacro_file   = pkg_share + '/description/robot.urdf.xacro'

    # -------------------------------------------------------------------------
    # Robot State Publisher
    # Only launched here when NOT in sim mode.
    # In sim mode, gz.launch.py owns the robot_state_publisher so it is
    # available before Gazebo spawns the entity.
    # -------------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        condition=UnlessCondition(sim_mode),
        parameters=[{
            'robot_description': ParameterValue(Command([
                'xacro', ' ', xacro_file,
                ' use_ros2_control:=', use_ros2_control,
                ' sim_mode:=', sim_mode
            ]), value_type=str)
        }]
    )

    # -------------------------------------------------------------------------
    # Gazebo simulation (sim mode only)
    # gz.launch.py starts Ignition Gazebo, spawns the robot, bridges topics,
    # and activates diff_drive_controller + joint_state_broadcaster.
    # This is what publishes the odom -> base_link TF needed by Nav2.
    # -------------------------------------------------------------------------
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gz.launch.py')
        ),
        launch_arguments={
            'use_sim_time':    use_sim_time,
            'use_ros2_control': use_ros2_control,
            'use_slam':        use_slam,
        }.items(),
        condition=IfCondition(sim_mode),
    )

    # -------------------------------------------------------------------------
    # ros2_control node + controller spawners (real-robot mode only)
    # In sim mode these are handled inside gz.launch.py via TimerAction.
    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    # Joystick / teleop
    # -------------------------------------------------------------------------
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # -------------------------------------------------------------------------
    # SLAM Toolbox (mapping mode, use_slam:=true only)
    # -------------------------------------------------------------------------
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  slam_params_file,
        }.items(),
        condition=IfCondition(use_slam),
    )

    # -------------------------------------------------------------------------
    # AMCL localization + Nav2 (use_slam:=true gates these for now —
    # see Issue 3: SLAM and AMCL should be mutually exclusive in a future fix)
    # -------------------------------------------------------------------------
    amcl_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file         = os.path.join(pkg_share, 'maps', 'map_save.yaml')

    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  amcl_params_file,
            'map':          map_file,
        }.items(),
        condition=IfCondition(use_slam),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_slam),
    )

    # -------------------------------------------------------------------------
    # Twist mux
    # -------------------------------------------------------------------------
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    # -------------------------------------------------------------------------
    # Optional YOLO object detection node (set ULTRALYTICS=true env var)
    # -------------------------------------------------------------------------
    launch_description = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='true',
            description='Run in simulation mode (starts Gazebo). Set false for real robot.'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control hardware interface'),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Launch SLAM, AMCL and Nav2'),
        # Sim-mode: Gazebo handles everything (RSP, control, odom TF)
        gazebo_sim,
        # Real-robot mode: RSP + control stack launched here
        robot_state_publisher,
        control_node,
        diff_drive_spawner,
        joint_broad_spawner,
        # Common nodes (both modes)
        joystick,
        slam,
        amcl,
        navigation,
        twist_mux,
    ]

    if os.environ.get('ULTRALYTICS', 'false') == 'true':
        import torch
        detector_node = Node(
            package='yolo_detection',
            executable='detector',
            name='yolo_node',
            parameters=[{
                'model':             'yolov10m.pt',
                'tracker':           'bytetrack.yaml',
                'device':            'cuda' if torch.cuda.is_available() else 'cpu',
                'enable':            True,
                'threshold':         0.5,
                'image_reliability': 1,
                'to_posestamped':    True,
                'to_pointcloud':     True,
                'visualize':         True,
                'stereo_vision':     True,
            }],
        )
        launch_description.append(detector_node)

    return LaunchDescription(launch_description)
