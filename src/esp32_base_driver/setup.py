from glob import glob

from setuptools import setup

package_name = "esp32_base_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Cau noi serial ROS 2 <-> ESP32-S3 diff drive base",
    license="MIT",
    entry_points={
        "console_scripts": [
            "base_node = esp32_base_driver.base_node:main",
            "cmd_vel_test = esp32_base_driver.cmd_vel_test:main",
            "odom_watch = esp32_base_driver.odom_watch:main",
            "calib_drive = esp32_base_driver.calib_drive:main",
            'traj_follow = esp32_base_driver.traj_follow:main',
        ],
    },
)
