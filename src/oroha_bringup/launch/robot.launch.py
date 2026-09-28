"""Bring up the OROHA base (ros2_control twin diff-drive) plus power measurement and IMU.

  ros2 launch oroha_bringup robot.launch.py                       # real hardware
  ros2 launch oroha_bringup robot.launch.py use_mock_hardware:=true power:=false   # no devices
  ros2 launch oroha_bringup robot.launch.py imu:=true rviz:=true

Connection settings (port, motor ids, reverse, counts_per_rev) live in
config/oroha_controllers.yaml under `mdrobot_hardware`; this launch injects them
into the URDF as xacro args (same mechanism as mdrobot_ros2_control's
bringup.launch.py). Drive with geometry_msgs/TwistStamped on /diff_cont/cmd_vel:

  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \\
      -r /cmd_vel:=/diff_cont/cmd_vel -p stamped:=true -p frame_id:=base_link -p speed:=0.2 -p turn:=0.5
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_hardware_params(controllers_path):
    """Read the `mdrobot_hardware` connection section from the controllers yaml."""
    with open(controllers_path) as f:
        data = yaml.safe_load(f) or {}
    section = data.get("mdrobot_hardware", {}) or {}
    return dict(section.get("ros__parameters", {}) or {})


def _truthy(s):
    return str(s).strip().lower() in ("true", "1", "yes")


def launch_setup(context, *args, **kwargs):
    use_mock = _truthy(LaunchConfiguration("use_mock_hardware").perform(context))
    controllers = LaunchConfiguration("controllers_file").perform(context)
    port_override = LaunchConfiguration("port").perform(context)

    hw = _load_hardware_params(controllers)
    if port_override:
        hw["port"] = port_override
    hw["use_mock_hardware"] = use_mock

    desc_share = FindPackageShare("oroha_description").perform(context)
    xacro_file = os.path.join(desc_share, "urdf", "oroha.urdf.xacro")
    xacro_cmd = [FindExecutable(name="xacro"), " ", xacro_file]
    for key, value in hw.items():
        sval = str(value).lower() if isinstance(value, bool) else str(value)
        xacro_cmd += [f" {key}:=", sval]
    # value_type=str: the URDF is not a valid YAML scalar, launch_ros would abort
    robot_description = {
        "robot_description": ParameterValue(Command(xacro_cmd), value_type=str)
    }

    bringup_share = FindPackageShare("oroha_bringup").perform(context)

    # controller_manager (4.48) takes the robot description from the robot_description
    # topic published by robot_state_publisher; it does not need the parameter.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )
    nodes = [
        control_node,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["diff_cont", "-c", "/controller_manager"],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="log",
            arguments=["-d", os.path.join(bringup_share, "rviz", "oroha.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]

    # Safety net (plan review C3): the MD400 has no communication watchdog and the
    # v1.4.0 plugin does not stop the motors on shutdown, so whenever
    # ros2_control_node exits — launch Ctrl-C or a crash — send VEL_CMD 0 / stop /
    # torque off to both controllers from this launch process. OpaqueFunction runs
    # in-process, so it also runs while launch is shutting down (new processes don't).
    if not use_mock and _truthy(LaunchConfiguration("safety_stop").perform(context)):
        port = str(hw.get("port", "/dev/oroha_md400"))

        def _safety_stop(ctx, *args, **kwargs):
            from oroha_tools.md_stop import stop_all
            lines = []
            res = stop_all(port, log=lines.append)
            return [LogInfo(msg="[oroha safety] ros2_control_node exited -> MD400 stop: "
                                + "; ".join(lines) + ("" if all(v == "ok" for v in res.values())
                                                      else "  !! NOT ALL STOPPED — use the E-stop"))]

        nodes.append(RegisterEventHandler(OnProcessExit(
            target_action=control_node, on_exit=[OpaqueFunction(function=_safety_stop)])))

    if _truthy(LaunchConfiguration("power").perform(context)):
        power_share = FindPackageShare("oroha_power").perform(context)
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(power_share, "launch", "power.launch.py")),
            launch_arguments={
                "params_file": os.path.join(bringup_share, "config", "power.yaml"),
                "simulate": "true" if use_mock else "false",
            }.items(),
        ))

    if _truthy(LaunchConfiguration("imu").perform(context)):
        um7_share = FindPackageShare("um7_driver").perform(context)
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(um7_share, "launch", "um7.launch.py")),
            launch_arguments={
                "params_file": os.path.join(bringup_share, "config", "um7.yaml"),
            }.items(),
        ))

    return nodes


def generate_launch_description():
    default_controllers = os.path.join(
        get_package_share_directory("oroha_bringup"), "config", "oroha_controllers.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="true: mock_components, no devices"),
        DeclareLaunchArgument("controllers_file", default_value=default_controllers,
                              description="controllers yaml (also carries mdrobot_hardware)"),
        DeclareLaunchArgument("port", default_value="",
                              description="RS485 port override; empty keeps the yaml value"),
        DeclareLaunchArgument("power", default_value="true",
                              description="launch oroha_power (Pico current/voltage)"),
        DeclareLaunchArgument("imu", default_value="false",
                              description="launch um7_driver"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="launch RViz with rviz/oroha.rviz"),
        DeclareLaunchArgument("safety_stop", default_value="true",
                              description="real hardware: stop both MD400 when ros2_control_node exits"),
        OpaqueFunction(function=launch_setup),
    ])
