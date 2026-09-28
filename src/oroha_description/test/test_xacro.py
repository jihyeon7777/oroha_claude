"""Render oroha.urdf.xacro in memory and check what the hardware plugin will receive.

Regression guard for the 2026-09-28 finding: macro-evaluated ${reverse_L} rendered as
"True", but mdrobot_system.cpp compares `reverse == "true"` case-sensitively, so the
left wheel would not have been reversed.
"""

import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro

URDF_DIR = Path(__file__).resolve().parents[1] / "urdf"


def render(mappings=None) -> ET.Element:
    text = (URDF_DIR / "oroha.urdf.xacro").read_text()
    text = text.replace("$(find oroha_description)/urdf", str(URDF_DIR))
    doc = minidom.parseString(text)
    xacro.process_doc(doc, mappings=mappings or {})
    return ET.fromstring(doc.toxml())


def hw_params(root):
    hw = root.find("ros2_control/hardware")
    return hw.findtext("plugin"), {p.get("name"): p.text for p in hw.findall("param")}


def joint_params(root, name):
    j = root.find(f"ros2_control/joint[@name='{name}']")
    params = {p.get("name"): p.text for p in j.findall("param")}
    cmds = [c.get("name") for c in j.findall("command_interface")]
    states = [s.get("name") for s in j.findall("state_interface")]
    return params, cmds, states


REAL = {"use_mock_hardware": "false", "port": "/dev/oroha_md400", "baudrate": "19200",
        "motor_id_L": "2", "motor_id_R": "1", "reverse_L": "true", "reverse_R": "false",
        "counts_per_rev_L": "30.0", "counts_per_rev_R": "30.0", "use_limit_sw": "0",
        "auto_enable": "true"}


def test_real_hardware_params_are_exact_strings():
    root = render(REAL)
    plugin, hw = hw_params(root)
    assert plugin == "mdrobot_ros2_control/MdrobotSystemHardware"
    assert hw["device_type"] == "twin"
    assert hw["port"] == "/dev/oroha_md400"
    assert hw["baudrate"] == "19200"
    assert hw["use_limit_sw"] == "0"
    assert hw["auto_enable"] == "true"
    left, _, _ = joint_params(root, "motor_L")
    right, _, _ = joint_params(root, "motor_R")
    assert left == {"motor_id": "2", "reverse": "true", "counts_per_rev": "30.0"}
    assert right == {"motor_id": "1", "reverse": "false", "counts_per_rev": "30.0"}


@pytest.mark.parametrize("mode", ["true", "false"])
def test_interfaces(mode):
    root = render({**REAL, "use_mock_hardware": mode})
    for name in ("motor_L", "motor_R"):
        _, cmds, states = joint_params(root, name)
        assert cmds == ["velocity"]
        assert states == ["position", "velocity", "effort"]


def test_mock_plugin():
    plugin, hw = hw_params(render({**REAL, "use_mock_hardware": "true"}))
    assert plugin == "mock_components/GenericSystem"
    assert hw.get("calculate_dynamics") == "true"


def test_defaults_match_verified_values():
    """Running xacro by hand (no args) must still give the verified OROHA wiring."""
    root = render()
    _, hw = hw_params(root)
    assert hw["use_limit_sw"] == "0"
    left, _, _ = joint_params(root, "motor_L")
    right, _, _ = joint_params(root, "motor_R")
    assert (left["motor_id"], left["reverse"]) == ("2", "true")
    assert (right["motor_id"], right["reverse"]) == ("1", "false")


def test_tree_single_root_and_wheel_axes():
    root = render(REAL)
    links = {link.get("name") for link in root.findall("link")}
    children = {j.find("child").get("link") for j in root.findall("joint")}
    roots = links - children
    assert roots == {"base_footprint"}
    for name, y_sign in (("motor_L", 1), ("motor_R", -1)):
        j = root.find(f"joint[@name='{name}']")
        assert j.get("type") == "continuous"
        assert j.find("axis").get("xyz") == "0 1 0"
        y = float(j.find("origin").get("xyz").split()[1])
        assert y_sign * y == pytest.approx(0.2255)
