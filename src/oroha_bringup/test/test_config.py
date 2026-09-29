"""Configuration rules for the OROHA base, checked without starting ROS.

Chain under test: config/oroha_controllers.yaml (mdrobot_hardware section) -> the same
key:=value mapping robot.launch.py builds -> oroha.urdf.xacro -> plugin parameters.
"""

import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

PKG = Path(__file__).resolve().parents[1]
URDF_DIR = PKG.parent / "oroha_description" / "urdf"
CFG = yaml.safe_load((PKG / "config" / "oroha_controllers.yaml").read_text())


def launch_mappings(hw: dict, use_mock: bool) -> dict:
    """Mirror of robot.launch.py: booleans lower-cased, everything else str()."""
    out = {k: (str(v).lower() if isinstance(v, bool) else str(v)) for k, v in hw.items()}
    out["use_mock_hardware"] = "true" if use_mock else "false"
    return out


def render(mappings) -> ET.Element:
    text = (URDF_DIR / "oroha.urdf.xacro").read_text()
    text = text.replace("$(find oroha_description)/urdf", str(URDF_DIR))
    doc = minidom.parseString(text)
    xacro.process_doc(doc, mappings=mappings)
    return ET.fromstring(doc.toxml())


def test_yaml_to_plugin_params_real():
    hw = CFG["mdrobot_hardware"]["ros__parameters"]
    root = render(launch_mappings(hw, use_mock=False))
    joints = {j.get("name"): {p.get("name"): p.text for p in j.findall("param")}
              for j in root.findall("ros2_control/joint")}
    assert joints["motor_L"]["reverse"] == "true"
    assert joints["motor_R"]["reverse"] == "false"
    assert joints["motor_L"]["motor_id"] == "2"
    assert joints["motor_R"]["motor_id"] == "1"
    hwp = {p.get("name"): p.text for p in root.findall("ros2_control/hardware/param")}
    assert hwp["use_limit_sw"] == "1", "use_limit_sw must be 1: the E-stop acts only through the CTRL stop gates"


def test_base_frame_is_urdf_root():
    root = render(launch_mappings(CFG["mdrobot_hardware"]["ros__parameters"], use_mock=True))
    links = {link.get("name") for link in root.findall("link")}
    children = {j.find("child").get("link") for j in root.findall("joint")}
    (urdf_root,) = links - children
    assert CFG["diff_cont"]["ros__parameters"]["base_frame_id"] == urdf_root


def test_controller_rules():
    cm = CFG["controller_manager"]["ros__parameters"]
    dc = CFG["diff_cont"]["ros__parameters"]
    assert cm["update_rate"] == 10
    assert "wheel_vel_cont" not in cm, "forward_command_controller has no command timeout"
    assert dc["publish_rate"] <= cm["update_rate"]
    assert dc["cmd_vel_timeout"] <= 0.5
    assert dc["publish_limited_velocity"] is True
    for axis in (dc["linear"]["x"], dc["angular"]["z"]):
        assert "min_acceleration" not in axis, "deprecated in diff_drive_controller 4.42"
        assert axis["max_deceleration"] <= 0.0
        assert axis["max_acceleration"] > 0.0
    assert dc["linear"]["x"]["max_velocity"] <= 1.0


@pytest.mark.parametrize("key", ["motor_id_L", "motor_id_R", "reverse_L", "reverse_R",
                                 "counts_per_rev_L", "counts_per_rev_R", "use_limit_sw"])
def test_verified_values_unchanged(key):
    verified = {"motor_id_L": 2, "motor_id_R": 1, "reverse_L": True, "reverse_R": False,
                "counts_per_rev_L": 30.0, "counts_per_rev_R": 30.0, "use_limit_sw": 1}
    assert CFG["mdrobot_hardware"]["ros__parameters"][key] == verified[key]
