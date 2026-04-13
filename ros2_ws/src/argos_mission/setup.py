from pathlib import Path

from setuptools import setup

package_name = "argos_mission"
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
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Argos Team",
    maintainer_email="argos@gatech.edu",
    description="Mission and perception bench nodes for the Argos SAR stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "argos-thermal = argos_mission.thermal_node:main",
            "argos-victim-detector = argos_mission.victim_detector_node:main",
            "argos-gas-mapping = argos_mission.gas_mapping_node:main",
            "argos-mission-orchestrator = argos_mission.mission_orchestrator_node:main",
        ],
    },
)
