from setuptools import setup

package_name = "orion_pybullet_sim"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/pybullet_sim.launch.py"]),
    ],
    install_requires=["setuptools", "pybullet"],
    zip_safe=True,
    maintainer="Orion",
    maintainer_email="user@example.com",
    description="PyBullet simulation controller for Orion robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pybullet_controller = orion_pybullet_sim.pybullet_controller_node:main",
        ],
    },
)
