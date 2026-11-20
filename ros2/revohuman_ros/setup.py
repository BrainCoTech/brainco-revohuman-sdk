from setuptools import setup

package_name = "revohuman_ros"

setup(
    name=package_name,
    version="0.1.4",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RevoHuman",
    maintainer_email="devnull@example.com",
    description="Thin ROS2 wrapper around the RevoHuman glove SDK (glove_node / bimanual_node, can embed Tracker poses and the wrist camera)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "glove_node = revohuman_ros.glove_node:main",
            "bimanual_node = revohuman_ros.bimanual_node:main",
        ],
    },
)
