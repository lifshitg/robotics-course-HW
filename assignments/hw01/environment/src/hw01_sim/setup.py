from glob import glob
from pathlib import Path
from setuptools import setup

setup(
    name='hw01_sim', version='0.1.0', packages=['hw01_sim'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hw01_sim']),
        ('share/hw01_sim', ['package.xml']),
        *[(f'share/hw01_sim/{folder}', [file for file in glob(f'{folder}/*') if Path(file).is_file()])
          for folder in ['launch', 'config', 'urdf', 'worlds']],
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Robotics course staff', maintainer_email='course@example.invalid',
    description='HW1 planar arm environment', license='Apache-2.0',
    entry_points={'console_scripts': [
        'motion_helper = hw01_sim.motion:main',
        'readiness_demo = hw01_sim.readiness:main',
        'check_reach = hw01_sim.check_reach:main',
    ]},
)
