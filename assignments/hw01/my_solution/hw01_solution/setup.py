from setuptools import find_packages, setup

package_name = 'hw01_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ido Fraiberg',
    maintainer_email='ido@todo.todo',
    description='Camera-guided reaching for HW01',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'reach_marker = hw01_solution.reach_marker:main'
        ],
    },
)
