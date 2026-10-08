from setuptools import setup

package_name = "revohuman_ros"

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
    maintainer="RevoHuman",
    maintainer_email="devnull@example.com",
    description="RevoHuman 手套 SDK 的 ROS2 薄壳（glove_node / bimanual_node，可嵌 tracker 位姿与腕部相机）",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "glove_node = revohuman_ros.glove_node:main",
            "bimanual_node = revohuman_ros.bimanual_node:main",
        ],
    },
)
