import os
import logging

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


logger = logging.getLogger('launch')


def _resolve_rviz_path(context):
    """Resolve the 'rviz_config' launch arg to an absolute .rviz file path.


    Accepts two forms:
      1. A plain name (no path separators, no extension), e.g. 'default'
         -> resolved to <pkg_share>/rviz/<name>.rviz
      2. An absolute path, e.g. '/tmp/my_arena.rviz'
         -> used as-is


    Raises FileNotFoundError if the resolved path does not exist so the
    error is clear rather than Ignition silently loading an empty world.
    """
    rviz_arg = LaunchConfiguration('rviz_config').perform(context)


    if os.path.isabs(rviz_arg):
        rviz_file = rviz_arg
    else:
        pkg_share = get_package_share_directory('mobile_robot')
        name = rviz_arg if rviz_arg.endswith('.rviz') else f'{rviz_arg}.rviz'
        rviz_file = os.path.join(pkg_share, 'rviz', name)


    if not os.path.isfile(rviz_file):
        raise FileNotFoundError(
            f"[rviz.launch.py] RViz config file not found: '{rviz_file}'\n"
            f"  Searched for RViz config arg value: '{rviz_arg}'\n"
            f"  Available RViz configs in package:\n"
            + '\n'.join(
                f'    {f}' for f in os.listdir(
                    os.path.join(get_package_share_directory('mobile_robot'), 'rviz')
                ) if f.endswith('.rviz')
            )
        )

    logger.info(f'[rviz.launch.py] Loading RViz config: {rviz_file}')
    return rviz_file


def launch_rviz(context, *args, **kwargs):
    """Launch RViz with the resolved config file path."""
    rviz_file = _resolve_rviz_path(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [rviz_node]


def generate_launch_description():

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='default',
            description='RViz config name (without .rviz) from the rviz/ directory'),
        DeclareLaunchArgument(
            'robot_description',
            default_value='true',
            description='Launch the robot_description node if true'),
        OpaqueFunction(function=launch_rviz),
    ])

    # Conditionally add YOLO visualiser when ULTRALYTICS=true in the environment
    if os.environ.get('ULTRALYTICS', 'false').lower() == 'true':
        viz_node = Node(
            package='yolo_detection',
            executable='visualizer',
            name='viz_node',
            parameters=[{
                'image_reliability': 1,
                'enable': True,
                'log_image': True,
            }],
        )
        launch_description.add_action(viz_node)

    return launch_description
