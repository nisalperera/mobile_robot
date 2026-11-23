"""Isolated four-wheel simulation bring-up. No command publishers."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WHEELS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)


def setup(context):
    share = Path(get_package_share_directory("mobile_robot"))
    source_root = Path(__file__).resolve().parents[1]
    controller_file = source_root / "config" / "mecanum_bringup_test.yaml"

    if not controller_file.is_file():
        raise RuntimeError(f"Missing controller configuration: {controller_file}")

    document = xacro.process_file(
        str(share / "description" / "robot.urdf.xacro"),
        mappings={
            "sim_mode": "true",
            "use_ros2_control": "true",
        },
    )
    robot = ET.fromstring(document.toxml())

    # Preserve the Fortress extension prefix through ElementTree serialization.
    ignition_namespace = "http://gazebosim.org/schema"
    ET.register_namespace("ignition", ignition_namespace)
    expressed_in = "{" + ignition_namespace + "}expressed_in"

    # Fixed-joint reduction retains base_footprint as a physical link.
    directions = robot.findall(".//fdir1")
    if len(directions) != 4:
        raise RuntimeError(f"Expected four friction directions, found {len(directions)}")
    for direction in directions:
        if direction.get(expressed_in) != "chassis":
            raise RuntimeError("Unexpected original friction reference")
        direction.set(expressed_in, "base_footprint")

        # Match the simulated roller handedness to the controller convention.
        vector = [float(value) for value in (direction.text or "").split()]
        if (
            len(vector) != 3
            or abs(vector[0] - 0.70710678) > 1e-6
            or abs(abs(vector[1]) - 0.70710678) > 1e-6
            or abs(vector[2]) > 1e-9
        ):
            raise RuntimeError(f"Unexpected original friction vector: {direction.text}")
        direction.text = f"{vector[0]:.8f} {-vector[1]:.8f} {vector[2]:.8f}"

    # Diagnostic collision proxy only; preserve visuals, mass and inertia.
    collision_width = float(
        LaunchConfiguration("wheel_collision_width").perform(context)
    )
    if not (0.0 < collision_width <= 0.025):
        raise RuntimeError("wheel_collision_width must be > 0 and <= 0.025 m")

    for joint_name in WHEELS:
        joint_element = robot.find(f"joint[@name='{joint_name}']")
        if joint_element is None:
            raise RuntimeError(f"Missing wheel joint: {joint_name}")
        wheel_name = joint_element.find("child").get("link")
        cylinders = robot.findall(
            f"link[@name='{wheel_name}']/collision/geometry/cylinder"
        )
        if len(cylinders) != 1:
            raise RuntimeError(f"Expected one cylinder collision for {wheel_name}")
        cylinder = cylinders[0]
        if abs(float(cylinder.get("length")) - 0.025) > 1e-9:
            raise RuntimeError(f"Unexpected original collision width for {wheel_name}")
        if abs(float(cylinder.get("radius")) - 0.024) > 1e-9:
            raise RuntimeError(f"Unexpected collision radius for {wheel_name}")
        cylinder.set("length", str(collision_width))

    systems = robot.findall("ros2_control")
    if len(systems) != 1:
        raise RuntimeError(f"Expected one ros2_control system, found {len(systems)}")

    system = systems[0]
    plugin = system.find("hardware/plugin")
    if plugin is None or plugin.text.strip() != "mecanum_ign_hardware/MecanumSystem":
        raise RuntimeError("Unexpected original simulation hardware plugin")

    plugin.text = "mecanum_four_wheel_hardware/FourWheelSystem"
    system.set("name", "FourWheelTestSystem")

    for joint in list(system.findall("joint")):
        system.remove(joint)

    for name in WHEELS:
        joint = ET.SubElement(system, "joint", {"name": name})
        command = ET.SubElement(joint, "command_interface", {"name": "velocity"})
        ET.SubElement(command, "param", {"name": "min"}).text = "-20"
        ET.SubElement(command, "param", {"name": "max"}).text = "20"
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})

    loaders = [
        element
        for element in robot.findall("gazebo/plugin")
        if element.get("filename") == "libgz_ros2_control-system.so"
    ]
    if len(loaders) != 1:
        raise RuntimeError(f"Expected one Gazebo control loader, found {len(loaders)}")

    loader = loaders[0]
    for parameter in list(loader.findall("parameters")):
        loader.remove(parameter)
    ET.SubElement(loader, "parameters").text = str(controller_file)

    if robot.find("link[@name='mecanum_test_center']") is not None:
        raise RuntimeError("Test frame already exists")

    ET.SubElement(robot, "link", {"name": "mecanum_test_center"})
    center_joint = ET.SubElement(
        robot, "joint", {"name": "mecanum_test_center_joint", "type": "fixed"}
    )
    ET.SubElement(center_joint, "parent", {"link": "chassis"})
    ET.SubElement(center_joint, "child", {"link": "mecanum_test_center"})
    ET.SubElement(center_joint, "origin", {"xyz": "0.15 0 0", "rpy": "0 0 0"})

    # Reuse only the existing simulator-launch function, not its spawners,
    # bridges, sensor relays, or LaunchDescription.
    spec = importlib.util.spec_from_file_location(
        "baseline_gazebo_launcher", str(share / "launch" / "gz.launch.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    gazebo_actions = module.launch_gazebo(context)

    return gazebo_actions + [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": ET.tostring(robot, encoding="unicode"),
            }],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            ],
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "robot_description", "-name", "mobile_robot"],
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "120",
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "mecanum_controller",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "120",
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="house_world"),
        DeclareLaunchArgument("wheel_collision_width", default_value="0.025"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=setup),
    ])