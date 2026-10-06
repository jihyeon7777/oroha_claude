from setuptools import find_packages, setup

package_name = "oroha_tools"

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
    description="OROHA operator tools",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "oroha_preflight = oroha_tools.preflight:main",
            "oroha_versions = oroha_tools.versions:main",
            "oroha_md_stop = oroha_tools.md_stop:main",
            "oroha_direction_check = oroha_tools.direction_check:main",
            "oroha_wheel_push = oroha_tools.wheel_push:main",
            "oroha_export_csv = oroha_tools.export_csv:main",
            "oroha_ledger = oroha_tools.ledger:main",
            "oroha_paper_export = oroha_tools.paper_export:main",
            "oroha_verify_export = oroha_tools.verify_export:main",
            "oroha_bus_probe = oroha_tools.bus_probe:main",
            "oroha_hw_recover = oroha_tools.hw_recover:main",
        ],
    },
)
