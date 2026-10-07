from pathlib import Path
from setuptools import setup

setup(
    name='hw02_sim', version='0.1.0', packages=['hw02_sim'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hw02_sim']),
        ('share/hw02_sim', ['package.xml']),
        *[(f'share/hw02_sim/{folder}', [str(p) for p in Path(folder).iterdir() if p.is_file()])
          for folder in ['launch', 'config']],
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Robotics course staff', maintainer_email='course@example.invalid',
    description='HW2 simulation and motion infrastructure', license='Apache-2.0',
    entry_points={'console_scripts': [
        'map_server = hw02_sim.map_server:main',
        'motion_helper = hw02_sim.motion:main',
        'go_to = hw02_sim.cli:main',
        'readiness_demo = hw02_sim.readiness:main',
        'check_navigation = hw02_sim.check_navigation:main',
    ]},
)
