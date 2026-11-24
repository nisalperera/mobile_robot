"""Reusable launch actions for mobile_robot."""

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def frame_fixer_node(
    *,
    name,
    kind,
    source,
    target,
    frame,
    use_sim_time,
):
    """Create a separately managed frame-fixer process."""
    if kind not in ('Image', 'Imu', 'LaserScan'):
        raise ValueError(f'Unsupported frame-fixer message type: {kind}')

    return Node(
        package='mobile_robot',
        executable='frame_fixer',
        name=name,
        arguments=[
            name,
            kind,
            source,
            target,
            frame,
        ],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        output='screen',
    )
