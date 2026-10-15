from glob import glob

from setuptools import setup

package_name = "revohuman_bringup"

setup(
    name=package_name,
    version="0.1.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RevoHuman",
    maintainer_email="devnull@example.com",
    description="RevoHuman SDK 的唯一 ROS2 入口（launch + 参数默认值）",
    license="Proprietary",
)
