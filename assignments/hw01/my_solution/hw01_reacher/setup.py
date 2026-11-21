from setuptools import find_packages, setup

package_name = 'hw01_reacher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Noa Shavit',
    maintainer_email='96386484+shavitnoa@users.noreply.github.com',
    description='HW1: find the red marker with the RGB-D camera and hover the arm tip above it.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'marker_reacher = hw01_reacher.marker_reacher:main',
            'measure_marker = hw01_reacher.measure_marker:main',
        ],
    },
)
