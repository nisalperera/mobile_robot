import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    
    package_name = 'mobile_robot'
    
    xacro_file = get_package_share_directory(package_name) + '/description/robot.urdf.xacro'


	# RViz
    rviz_config_file = get_package_share_directory(package_name) + "/config/camera.rviz"
    rviz_node = Node(package='rviz2',
					 executable='rviz2',
					 name='rviz2',
					 output='log',
					 arguments=['-d', rviz_config_file])
    

    # YOLO detections
    model_type = LaunchConfiguration("model_type")
    model_type_cmd = DeclareLaunchArgument(
        "model_type",
        default_value="YOLO",
        choices=["YOLO", "NAS"],
        description="Model type form Ultralytics (YOLO, NAS")

    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value="yolov8m.pt",
        description="Model name or path")

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Device to use (GPU/CPU)")

    enable = LaunchConfiguration("enable")
    enable_cmd = DeclareLaunchArgument(
        "enable",
        default_value="True",
        description="Whether to start YOLOv8 enabled")

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument(
        "threshold",
        default_value="0.5",
        description="Minimum probability of a detection to be published")

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/camera/rgb/image_raw",
        description="Name of the input image topic")

    image_reliability = LaunchConfiguration("image_reliability")
    image_reliability_cmd = DeclareLaunchArgument(
        "image_reliability",
        default_value="2",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)")

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="yolo",
        description="Namespace for the nodes")
    
    detector_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    "yolo_bringup"), "launch", "yolov8.launch.py")),
            launch_arguments={
                "model": LaunchConfiguration("model", default="yolov10m.pt"),
                "tracker": LaunchConfiguration("tracker", default="bytetrack.yaml"),
                "device": LaunchConfiguration("device", default="cuda:0"),
                "enable": LaunchConfiguration("enable", default="True"),
                "threshold": LaunchConfiguration("threshold", default="0.5"),
                "input_image_topic": LaunchConfiguration("input_image_topic", default="/camera/rgb/image_raw"),
                "image_reliability": LaunchConfiguration("image_reliability", default="2"),
                "namespace": LaunchConfiguration("namespace", default="yolo"),
            }.items(),
        )


	
	# Robot State Publisher 
    robot_state_publisher = Node(package='robot_state_publisher',
								 executable='robot_state_publisher',
								 name='robot_state_publisher',
								 output='both',
								 parameters=[{'robot_description': Command(['xacro', ' ', xacro_file])           
								}])


	# Joint State Publisher 
    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
									executable='joint_state_publisher_gui',
									output='screen',
									name='joint_state_publisher_gui')


    return LaunchDescription([
        robot_state_publisher, 
        joint_state_publisher_gui, 
        rviz_node, 
        detector_node,
    ])
	# return LaunchDescription([robot_state_publisher, rviz_node ])
