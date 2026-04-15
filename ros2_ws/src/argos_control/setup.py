# ROS 2 package setup for argos_control.
# The source files live in ros2_ws/argos_control/ — this file just tells colcon where to find them.

from pathlib import Path

from setuptools import setup

package_name = "argos_control"
# Source lives two levels up in ros2_ws/argos_control/ rather than inside src/
package_source = Path(__file__).resolve().parents[2] / package_name

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={package_name: str(package_source)},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "transforms3d"],
    zip_safe=True,
    maintainer="Argos Team",
    maintainer_email="argos@gatech.edu",
    description="Argos control math, configuration, and bench tools.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "argos-single-leg-test = argos_control.single_leg_test:main",
            "argos-command-mux = argos_control.command_mux_node:main",
            "argos-command-odometry = argos_control.command_odometry_node:main",
            "argos-gait-planner = argos_control.gait_planner_node:main",
            "argos-foothold-checker = argos_control.foothold_checker_node:main",
            "argos-safety = argos_control.safety_node:main",
            "argos-joint-command-publisher = argos_control.joint_command_publisher_node:main",
            "argos-demo-commander = argos_control.demo_commander_node:main",
        ],
    },
)
