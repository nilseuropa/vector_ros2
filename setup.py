from setuptools import setup

package_name = "vector_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/vector.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="ROS 2 wrapper for Anki Vector (wire-pod / OSKR)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vector_driver = vector_ros2.driver_node:main",
        ],
    },
)
