from setuptools import setup

setup(
    name='hw01_reach', version='0.1.0', packages=['hw01_reach'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hw01_reach']),
        ('share/hw01_reach', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='HW1 student', maintainer_email='student@example.invalid',
    description='HW1 camera-guided reaching node', license='Apache-2.0',
    entry_points={'console_scripts': [
        'reach_node = hw01_reach.reach_node:main',
    ]},
)
