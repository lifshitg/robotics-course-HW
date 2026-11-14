from setuptools import setup

package_name = 'hw01_reach'

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
    maintainer='Yuval Bublil',
    maintainer_email='yuvalbublil2@gmail.com',
    description='Camera-guided reaching node for HW1',
    license='MIT',
    entry_points={
        'console_scripts': [
            'reach = hw01_reach.reach:main',
        ],
    },
)
