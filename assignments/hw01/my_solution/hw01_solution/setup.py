from setuptools import setup

setup(
    name='hw01_solution', version='0.1.0', packages=['hw01_solution'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hw01_solution']),
        ('share/hw01_solution', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    description='HW1 camera-guided reaching', license='Apache-2.0',
    entry_points={'console_scripts': [
        'solution = hw01_solution.solution:main',
    ]},
)
