from setuptools import setup

package_name = "arm_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Prateek Singh",
    description="Cup/glass detection, target estimation, hand-eye calibration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            # TODO: "cup_detector = arm_vision.cup_detector_node:main",
        ],
    },
)
