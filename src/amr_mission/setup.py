from glob import glob

from setuptools import setup

package_name = "amr_mission"

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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="huy",
    maintainer_email="todo@todo.com",
    description="Mission manager cho AMR",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_manager = amr_mission.mission_manager:main",
            "station_eval = amr_mission.station_eval:main",
        ],
    },
)