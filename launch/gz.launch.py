import os
import logging

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
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

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    pkg_share = get_package_share_directory(package_name)
    xacro_file = pkg_share + '/description/robot.urdf.xacro'

    # -------------------------------------------------------------------------
    # FIX 1: robot_state_publisher must be present in THIS launch file.
    # Previously it was only conditionally launched from rviz.launch.py via
    # the robot_description argument. gz.launch.py needs its own RSP so that
    # /robot_description is published BEFORE Ignition spawns the entity and
    # BEFORE the controller_manager tries to read the URDF.
    # -------------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': Command([
                'xacro', ' ', xacro_file,
                ' use_ros2_control:=', use_ros2_control,
                ' sim_mode:=', use_sim_time,
            ]),
            'use_sim_time': use_sim_time,
        }],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': use_ros2_control,
            # robot_description=false so rviz.launch.py does NOT spawn a second RSP
            'robot_description': 'false',
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', package_name,
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # -------------------------------------------------------------------------
    # FIX 2: Bridge /tf as bidirectional so both Ignition-published TFs
    # (from ign_ros2_control) and ROS2-published TFs (from robot_state_publisher)
    # are available on the same /tf topic.
    # Format: topic@ros_type@ign_type  (@ = bidirectional)
    # Previously used [ (Ignition->ROS only), which drops RSP wheel TFs.
    # -------------------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            f'/model/{package_name}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            f'/model/{package_name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/left_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/left_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/right_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/right_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'expand_gz_topic_names': False},
        ],
    )

    # -------------------------------------------------------------------------
    # FIX 3: Spawn joint_state_broadcaster and diff_drive_controller.
    # Previously MISSING from gz.launch.py entirely.
    # Without these, controller_manager loads but no controller publishes
    # /joint_states, so robot_state_publisher cannot compute wheel TFs.
    # TimerAction(5.0s) allows Ignition + ign_ros2_control plugin to fully
    # initialise before spawner connects to the controller_manager service.
    # -------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Delay controller spawning until Ignition + ign_ros2_control are ready
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
            description='Use SLAM if true'),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo server-only (no GUI/rendering). '
                        'Set false once NVIDIA GPU passthrough is configured.'),
        OpaqueFunction(function=log_args),
        robot_state_publisher,
        OpaqueFunction(function=launch_gazebo),
        spawn_entity,
        bridge,
        rviz,
        delayed_controller_spawners,
    ])
