import os
from glob import glob

from setuptools import find_packages, setup

package_name = "oroha_power"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "calibration"), glob("config/calibration/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Taesu Yim",
    maintainer_email="taesuyim.kopo@gmail.com",
    description="Pico current/voltage measurement host node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "power_node = oroha_power.power_node:main",
        ],
    },
)
