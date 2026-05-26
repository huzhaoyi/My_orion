from setuptools import setup

package_name = "sealien_ctrlpilot_manipulator_orion_holoocean_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/holoocean_bridge.launch.py"]),
        ("share/" + package_name + "/config", ["config/holoocean_bridge_params.yaml", "config/panel_obstacles_mtc_fragment.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Orion",
    maintainer_email="user@example.com",
    description="Bridge HoloOcean ArmSensor to Orion joint_states for MTC",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "arm_sensor_to_joint_state = sealien_ctrlpilot_manipulator_orion_holoocean_bridge.arm_sensor_to_joint_state_node:main",
            "joint_state_web_relay = sealien_ctrlpilot_manipulator_orion_holoocean_bridge.joint_state_web_relay_node:main",
            "trajectory_to_agent_bridge = sealien_ctrlpilot_manipulator_orion_holoocean_bridge.trajectory_to_agent_bridge_node:main",
            "target_sensor_to_object_pose = sealien_ctrlpilot_manipulator_orion_holoocean_bridge.target_sensor_to_object_pose_node:main",
            "cable_sensor_to_object_pose = sealien_ctrlpilot_manipulator_orion_holoocean_bridge.cable_sensor_to_object_pose_node:main",
        ],
    },
)
