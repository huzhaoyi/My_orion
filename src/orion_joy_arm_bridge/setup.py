from setuptools import setup

package_name = "orion_joy_arm_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/joy_manipulator.launch.py"]),
        ("share/" + package_name + "/config", ["config/joy_manipulator.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Orion",
    maintainer_email="user@example.com",
    description="Joy to manipulator bridge for Orion MTC stack",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_manipulator_node = orion_joy_arm_bridge.joy_manipulator_node:main",
        ],
    },
)
