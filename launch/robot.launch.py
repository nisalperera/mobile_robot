import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'mobile_robot'
    
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_slam = LaunchConfiguration('use_slam')

    xacro_file = get_package_share_directory(package_name) + '/description/robot.urdf.xacro'

	# Robot State Publisher 
    robot_state_publisher = Node(package='robot_state_publisher',
								executable='robot_state_publisher',
								name='robot_state_publisher',
								output='both',
								parameters=[{
                                        'robot_description': Command(['xacro', ' ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])           
								    }],
                            )
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory(package_name),'launch','joystick.launch.py')]), 
                    launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="diff_drive_controller",
        arguments=["diff_drive_controller"],
        parameters=[{'use_sim_time': use_sim_time}]
        # namespace=package_name
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': use_sim_time}]
        # namespace=package_name
    )

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','controllers.yaml')
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controller_params_file,
            {"use_sim_time": use_sim_time},
            {"tf_buffer_duration": 10.0}],
        output="both",
        respawn=True,
        # namespace=package_name
    )

    slam_params_file = os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml')
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory(package_name),'launch','online_async_launch.py')]), 
                    launch_arguments={'use_sim_time': use_sim_time, 'params_file': slam_params_file}.items(),
                    condition=IfCondition(use_slam)
    )

    amcl_params_file = os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory(package_name),'maps','map_save.yaml')
    amcl = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory(package_name),'launch','localization.launch.py')]), 
                    launch_arguments={
                        'use_sim_time': use_sim_time, 
                        'params_file': amcl_params_file,
                        'map': map_file
                    }.items(),
                    condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [os.path.join(get_package_share_directory(package_name),'launch','navigation.launch.py')]), 
                        launch_arguments={'use_sim_time': use_sim_time}.items(), 
                        condition=IfCondition(use_slam)
    )

    ekf_params_file = os.path.join(get_package_share_directory(package_name),'config','ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node', 
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,  # Use full path to your ekf.yaml
            {"use_sim_time": use_sim_time}
        ]
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out','/diff_drive_controller/cmd_vel_unstamped')]
        )
    
    launch_description = [
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
            default_value='true',
            description='Use ros2_control if true'),
        robot_state_publisher, 
        joystick,
        control_node,
        diff_drive_spawner,
        joint_broad_spawner,
        # slam,
        amcl, 
        navigation,
        ekf_node,
        twist_mux,
    ]
    
    if os.environ.get("ULTRALYTICS", "false") == "true":
        import torch
        detector_node = Node(
                package='yolo_detection',
                executable='detector',
                name='yolo_node',
                parameters=[{
                    'model': 'yolov10m.pt',
                    'tracker': 'bytetrack.yaml',
                    'device': 'cuda' if torch.cuda.is_available() else 'cpu',
                    'enable': True,
                    'threshold': 0.5,
                    'image_reliability': 1,
                    'to_posestamped': True,
                    'to_pointcloud': True,
                    'visualize': True,
                    'stereo_vision': True
                }],
            )
        
        launch_description.append(detector_node)    

    return LaunchDescription(launch_description)