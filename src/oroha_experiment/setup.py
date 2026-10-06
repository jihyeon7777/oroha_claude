from setuptools import find_packages, setup

package_name = "oroha_experiment"

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
    description="OROHA paper experiments: path profiles, runner node, CLI",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "runner = oroha_experiment.runner_node:main",
            "oroha_exp = oroha_experiment.cli:entry",
            "oroha_profile = oroha_experiment.profiles:main",
        ],
    },
)
