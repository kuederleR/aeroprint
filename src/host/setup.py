from setuptools import setup
import os
from glob import glob

package_name = "host"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ryan",
    maintainer_email="ryankuederle@icloud.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_node = host.test_node:main",
            "test_sub = host.test_sub:main",
            "pc_processor = host.pc_processor:main",
        ],
    },
)
