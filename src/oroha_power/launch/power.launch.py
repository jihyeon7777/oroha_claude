"""Launch the Pico measurement node.

  ros2 launch oroha_power power.launch.py                      # /dev/oroha_pico
  ros2 launch oroha_power power.launch.py port:=/dev/ttyACM0
  ros2 launch oroha_power power.launch.py simulate:=true       # no device, synthetic frames
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    port = LaunchConfiguration("port").perform(context)
    simulate = LaunchConfiguration("simulate").perform(context).strip().lower()

    # later entries win; only override what was given on the command line.
    # Overrides are typed explicitly (a bare LaunchConfiguration would arrive as
    # a string and rclpy rejects a str for a bool parameter).
    parameters = [params_file]
    overrides = {}
    if port:
        overrides["port"] = port
    if simulate:
        overrides["simulate"] = simulate in ("true", "1", "yes")
    if overrides:
        parameters.append(overrides)

    return [Node(package="oroha_power", executable="power_node", name="oroha_power",
                 output="screen", parameters=parameters)]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("oroha_power"),
                                  "config", "power_default.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("port", default_value="", description="serial port override"),
        DeclareLaunchArgument("simulate", default_value="",
                              description="true/false override; empty keeps the yaml value"),
        OpaqueFunction(function=_setup),
    ])
