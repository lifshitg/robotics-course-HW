from setuptools import setup

package_name = "marker_reacher"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yuval Hadar, Moriel Turgeman",
    maintainer_email="todo@todo.todo",
    description="HW1 camera-guided reaching node",
    license="MIT",
    entry_points={
        "console_scripts": [
            "reacher_node = marker_reacher.reacher_node:main",
        ],
    },
)
