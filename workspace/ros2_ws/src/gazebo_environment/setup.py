import os
from glob import glob
from setuptools import find_packages, setup

package_name = "gazebo_environment"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),  # new
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MrSkyGodz",
    maintainer_email="yunus.akdal@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odometry_tf = gazebo_environment.odometry_tf:main",
            "pure_pursuit_controller = gazebo_environment.pure_pursuit_controller:main",    #new
            "pid_speed_controller = gazebo_environment.pid_speed_controller:main",          # new
            "integrated_controller = gazebo_environment.integrated_controller:main",        # new
            "pose_logger = gazebo_environment.pose_logger:main",
            "pure_pursuit = gazebo_environment.pure_pursuit:main",
            "PID_controller = gazebo_environment.PID_controller:main",

        ],
    },
)
