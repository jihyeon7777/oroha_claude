from setuptools import find_packages, setup

package_name = "oroha_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Taesu Yim",
    maintainer_email="taesuyim.kopo@gmail.com",
    description="Dead-man keyboard teleop for OROHA",
    license="Apache-2.0",
    entry_points={"console_scripts": ["deadman_teleop = oroha_teleop.deadman_teleop:main"]},
)
