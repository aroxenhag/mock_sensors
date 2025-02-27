from setuptools import setup

package_name = "mock_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="Mock sensor node for ROS 2 automation testing.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_sensor = mock_sensors.mock_sensor:main",
        ],
    },
)
