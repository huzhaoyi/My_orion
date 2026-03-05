from setuptools import setup

package_name = "orion_holoocean_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/holoocean_bridge.launch.py"]),
        ("share/" + package_name + "/config", ["config/holoocean_bridge_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Orion",
    maintainer_email="user@example.com",
    description="Bridge HoloOcean ArmSensor to Orion joint_states for MTC",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_sensor_to_joint_state = orion_holoocean_bridge.arm_sensor_to_joint_state_node:main",
            "trajectory_to_agent_bridge = orion_holoocean_bridge.trajectory_to_agent_bridge_node:main",
            "calibrate_arm_mapping = orion_holoocean_bridge.calibrate_arm_mapping_node:main",
            "arm_sensor_mapping_test = orion_holoocean_bridge.arm_sensor_mapping_test_node:main",
        ],
    },
)
