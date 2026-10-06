"""The shared environment; no student or TA implementation is launched here."""
from pathlib import Path
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('hw01_sim'))
    preset = int(LaunchConfiguration('preset').perform(context))
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    headless = LaunchConfiguration('headless_rendering').perform(context) == 'true'
    robot = xacro.process_file(str(share / 'urdf' / 'arm.urdf.xacro'),
                              mappings={'controllers': str(share / 'config' / 'controllers.yaml')}).toxml()
    x, y = {1: (0.35, -0.15), 2: (0.40, 0.0), 3: (0.35, 0.15)}[preset]
    marker = f'''<sdf version="1.9"><model name="marker"><static>true</static>
      <pose>{x} {y} 0.055 0 0 0</pose><link name="marker_link">
      <collision name="marker_collision"><geometry><cylinder><radius>0.018</radius><length>0.01</length></cylinder></geometry></collision>
      <visual name="visual"><geometry><cylinder><radius>0.018</radius><length>0.01</length></cylinder></geometry>
      <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>
      <sensor name="contacts" type="contact"><always_on>true</always_on><update_rate>30</update_rate>
      <topic>/hw01/marker_contacts</topic><contact><collision>marker_collision</collision></contact></sensor>
      </link></model></sdf>'''
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-string', robot, '-name', 'hw01_arm', '-allow_renaming', 'false'])
    states = Node(package='controller_manager', executable='spawner', output='screen',
                  arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'])
    arm = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['arm_controller', '--controller-manager-timeout', '120'])
    gz = Path(get_package_share_directory('ros_gz_sim'))
    rendering = '' if gui else '-s ' + ('--headless-rendering ' if headless else '')
    args = f'-r -v 2 {rendering}"{share / "worlds" / "hw01.sdf"}"'
    return [
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', str(Path(get_package_share_directory('hw01_gz_checks')).parents[1] / 'lib')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz / 'launch' / 'gz_sim.launch.py')),
                                 launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=lambda event, _: [states] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Arm creation failed'))])),
        RegisterEventHandler(OnProcessExit(target_action=states, on_exit=lambda event, _: [arm] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Joint state controller failed'))])),
        spawn,
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-string', marker, '-name', 'marker', '-allow_renaming', 'false', '-x', str(x), '-y', str(y), '-z', '0.055']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': str(share / 'config' / 'bridge.yaml'), 'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='world_to_camera',
             arguments=['--x', '0.30', '--z', '1.0', '--pitch', '1.5707963267948966',
                        '--frame-id', 'world', '--child-frame-id', 'camera_link'],
             parameters=[{'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_to_optical',
             arguments=['--roll', '-1.5707963267948966', '--yaw', '-1.5707963267948966',
                        '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame'],
             parameters=[{'use_sim_time': True}]),
        Node(package='hw01_sim', executable='motion_helper', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share / 'config' / 'view.rviz')], parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('preset', default_value='1', choices=['1', '2', '3']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('headless_rendering', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
