from setuptools import find_packages, setup

package_name = 'hw01_solution'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roi Prives',
    maintainer_email='prives@post.bgu.ac.il',
    description='HW1 camera-guided reaching',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reach_target = hw01_solution.reach_node:main',
            'capture_frame = hw01_solution.capture_frame:main',
        ],
    },
)
