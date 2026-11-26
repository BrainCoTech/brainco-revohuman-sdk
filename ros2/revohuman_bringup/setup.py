from glob import glob

from setuptools import setup

package_name = "revohuman_bringup"

setup(
    name=package_name,
    version="0.1.4",
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
    description="Single ROS2 entry point of the RevoHuman SDK (launch + parameter defaults)",
    license="Apache-2.0",
)
