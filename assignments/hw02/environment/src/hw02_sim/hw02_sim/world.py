"""Self-contained Gazebo world generated from the same layout as /map."""
from .scenario import LAYOUTS, START, GOAL, WIDTH, HEIGHT, PILLAR_RADIUS, ROBOT_RADIUS


def material(color):
    return f'<material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material>'


def solid(name, pose, geometry, color, static=True):
    return f'''<model name="{name}"><static>{str(static).lower()}</static><pose>{pose}</pose>
    <link name="{name}_link"><collision name="{name}_collision"><geometry>{geometry}</geometry></collision>
    <visual name="visual"><geometry>{geometry}</geometry>{material(color)}</visual></link></model>'''


def robot():
    links = f'''<link name="base_link"><pose>0 0 0.14 0 0 0</pose>
    <inertial><mass>3</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.094</izz></inertia></inertial>
    <collision name="robot_base_collision"><geometry><cylinder><radius>{ROBOT_RADIUS}</radius><length>0.10</length></cylinder></geometry>
    </collision>
    <sensor name="contacts" type="contact"><contact><collision>robot_base_collision</collision></contact></sensor>
    <visual name="body"><geometry><cylinder><radius>{ROBOT_RADIUS}</radius><length>0.10</length></cylinder></geometry>'''
    links += material('0.15 0.4 0.85') + '''</visual>
    <visual name="heading"><pose>0.17 0 0.056 0 0 0</pose><geometry><box><size>0.10 0.035 0.012</size></box></geometry>''' + material('1 0.8 0.1') + '</visual></link>'
    joints = ''
    for name, y in [('left', .17), ('right', -.17)]:
        links += f'''<link name="{name}_wheel"><pose>0 {y} 0.08 -1.5707963267948966 0 0</pose>
        <inertial><mass>0.2</mass><inertia><ixx>0.0004</ixx><iyy>0.0004</iyy><izz>0.00064</izz></inertia></inertial>
        <collision name="robot_{name}_collision"><geometry><cylinder><radius>0.08</radius><length>0.04</length></cylinder></geometry>
        <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision>
        <sensor name="contacts" type="contact"><contact><collision>robot_{name}_collision</collision></contact></sensor>
        <visual name="wheel"><geometry><cylinder><radius>0.08</radius><length>0.04</length></cylinder></geometry>{material('0.1 0.1 0.1')}</visual></link>'''
        joints += f'''<joint name="{name}_joint" type="revolute"><parent>base_link</parent><child>{name}_wheel</child>
        <axis><xyz>0 0 1</xyz></axis></joint>'''
    for name, x in [('front', .18), ('rear', -.18)]:
        links += f'''<link name="{name}_caster"><pose>{x} 0 0.025 0 0 0</pose>
        <inertial><mass>0.02</mass><inertia><ixx>0.000005</ixx><iyy>0.000005</iyy><izz>0.000005</izz></inertia></inertial>
        <collision name="robot_{name}_collision"><geometry><sphere><radius>0.025</radius></sphere></geometry>
        <surface><friction><ode><mu>0.001</mu><mu2>0.001</mu2></ode></friction></surface></collision>
        <sensor name="contacts" type="contact"><contact><collision>robot_{name}_collision</collision></contact></sensor>
        <visual name="caster"><geometry><sphere><radius>0.025</radius></sphere></geometry>{material('0.2 0.2 0.2')}</visual></link>'''
        joints += f'<joint name="{name}_joint" type="ball"><parent>base_link</parent><child>{name}_caster</child></joint>'
    return f'''<model name="hw02_robot"><pose>{START[0]} {START[1]} 0 0 0 0</pose>{links}{joints}
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>left_joint</left_joint><right_joint>right_joint</right_joint>
    <wheel_separation>0.34</wheel_separation><wheel_radius>0.08</wheel_radius>
    <topic>/model/hw02_robot/cmd_vel</topic><odom_publish_frequency>20</odom_publish_frequency>
    <max_linear_velocity>0.2</max_linear_velocity><min_linear_velocity>-0.2</min_linear_velocity>
    <max_angular_velocity>0.8</max_angular_velocity><min_angular_velocity>-0.8</min_angular_velocity>
    </plugin></model>'''


def world(layout):
    pillars = ''.join(solid(f'pillar_{i}', f'{x} {y} 0.5 0 0 0',
        f'<cylinder><radius>{PILLAR_RADIUS}</radius><length>1</length></cylinder>', '0.75 0.25 0.15')
        for i, (x, y) in enumerate(LAYOUTS[layout]))
    walls = ''.join(solid('wall_'+name, pose, f'<box><size>{size}</size></box>', '0.7 0.7 0.7')
        for name, pose, size in [
            ('left', f'-0.075 {HEIGHT/2} 0.5 0 0 0', f'0.15 {HEIGHT+.3} 1'),
            ('right', f'{WIDTH+.075} {HEIGHT/2} 0.5 0 0 0', f'0.15 {HEIGHT+.3} 1'),
            ('bottom', f'{WIDTH/2} -0.075 0.5 0 0 0', f'{WIDTH} 0.15 1'),
            ('top', f'{WIDTH/2} {HEIGHT+.075} 0.5 0 0 0', f'{WIDTH} 0.15 1')])
    goal = f'<model name="goal"><static>true</static><pose>{GOAL[0]} {GOAL[1]} 0.003 0 0 0</pose><link name="goal_link"><visual name="goal"><geometry><cylinder><radius>0.18</radius><length>0.005</length></cylinder></geometry>' + material('0.1 0.85 0.2') + '</visual></link></model>'
    return f'''<?xml version="1.0"?><sdf version="1.9"><world name="hw02">
    <physics name="physics" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="libhw02_observer.so" name="hw02::Observer"/>
    <scene><ambient>0.7 0.7 0.7 1</ambient><background>0.85 0.9 0.95 1</background></scene>
    <light name="sun" type="directional"><pose>0 0 8 0 0 0</pose><direction>0 0 -1</direction><diffuse>0.8 0.8 0.8 1</diffuse></light>
    {solid('ground', f'{WIDTH/2} {HEIGHT/2} -0.05 0 0 0', f'<box><size>{WIDTH+2} {HEIGHT+2} 0.1</size></box>', '0.9 0.9 0.9')}
    {walls}{pillars}{goal}{robot()}</world></sdf>'''
