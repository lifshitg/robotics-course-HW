from setuptools import setup

setup(
    name='hw01_reach',
    version='0.1.0',
    packages=['hw01_reach'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hw01_reach']),
        ('share/hw01_reach', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HW01 student',
    maintainer_email='student@example.invalid',
    description='Camera-guided marker reaching for HW01',
    license='Apache-2.0',
    entry_points={'console_scripts': ['reach_marker = hw01_reach.node:main']},
)
