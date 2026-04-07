from setuptools import setup

package_name = "aloha_rx64_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/rx64_single.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Openpi User",
    maintainer_email="user@example.com",
    description="Minimal ROS2 bridge for Dynamixel RX-64 on Orange Pi AI Pro",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rx64_single_node = aloha_rx64_bridge.rx64_node:main",
            "rx64_ping = aloha_rx64_bridge.rx64_ping:main",
        ],
    },
)
