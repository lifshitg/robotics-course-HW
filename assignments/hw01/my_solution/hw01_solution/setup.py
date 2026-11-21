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
    maintainer='Yoav Levin & Ziv Zaltsman',
    maintainer_email='yoavlevi@post.bgu.ac.il',
    description='HW1 solution: red marker point detector and arm motion nodes.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'point_detector = hw01_solution.point_detector:main',
            'arm_motion = hw01_solution.arm_motion:main',
        ],
    },
)