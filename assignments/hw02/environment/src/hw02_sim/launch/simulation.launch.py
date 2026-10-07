"""Launch only shared HW2 infrastructure; never import student/TA planners."""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from hw02_sim.world import world
from hw02_sim.scenario import LAYOUTS


def setup(context):
    layout = LaunchConfiguration('layout').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    share = Path(get_package_share_directory('hw02_sim'))
    plugin = Path(get_package_share_directory('hw02_gz_checks')).parents[1] / 'lib'
    gz = Path(get_package_share_directory('ros_gz_sim'))
    handle = tempfile.NamedTemporaryFile(prefix='hw02-', suffix='.sdf', mode='w', delete=False)
    handle.write(world(layout)); handle.close()
    args = f'-r -v 2 {"" if gui else "-s "}"{handle.name}"'
    def cleanup(_event, _context):
        Path(handle.name).unlink(missing_ok=True)
        return []
    return [
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', str(plugin)),
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz/'launch/gz_sim.launch.py')),
                                 launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': str(share/'config/bridge.yaml'), 'use_sim_time': True}]),
        Node(package='hw02_sim', executable='map_server', output='screen',
             parameters=[{'layout': layout, 'use_sim_time': True}]),
        Node(package='hw02_sim', executable='motion_helper', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share/'config/view.rviz')], parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='wide', choices=list(LAYOUTS)),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
