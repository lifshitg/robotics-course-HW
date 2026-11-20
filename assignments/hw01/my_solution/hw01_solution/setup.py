from setuptools import setup

package_name = 'hw01_solution'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HW01 student',
    maintainer_email='student@example.com',
    description='Camera-guided marker reaching for HW01',
    license='Apache-2.0',
    entry_points={
        'console_scripts': ['reach_marker = hw01_solution.reach_marker:main'],
    },
)
