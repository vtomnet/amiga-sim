"""Static validation tests for the Farm-NG Amiga + Kinova model."""

from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

REPO_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = REPO_ROOT / "models" / "amiga_kinova" / "model.sdf"
CONTROL_CONFIG_PATH = REPO_ROOT / "config" / "ros2_control.yaml"
WORLD_PATH = REPO_ROOT / "worlds" / "amiga_world.sdf"
LAUNCH_PATH = REPO_ROOT / "ros2" / "launch" / "amiga_sim.launch.py"


def _load_xml(path: Path) -> ET.Element:
    return ET.parse(path).getroot()


def test_model_contains_expected_links_and_joints() -> None:
    root = _load_xml(MODEL_PATH)

    link_names = {link.attrib.get("name") for link in root.findall(".//link")}
    expected_links = {
        "base_link",
        "left_wheel_link",
        "right_wheel_link",
        "kinova_base_link",
        "kinova_link_1",
        "kinova_link_2",
        "kinova_link_3",
        "kinova_link_4",
        "kinova_link_5",
        "kinova_link_6",
        "kinova_end_effector",
    }
    missing_links = expected_links.difference(link_names)
    assert not missing_links, f"Missing links: {missing_links}"

    joint_names = {joint.attrib.get("name") for joint in root.findall(".//joint")}
    expected_joints = {
        "left_wheel_joint",
        "right_wheel_joint",
        "kinova_joint_1",
        "kinova_joint_2",
        "kinova_joint_3",
        "kinova_joint_4",
        "kinova_joint_5",
        "kinova_joint_6",
        "kinova_joint_7",
        "kinova_tool_joint",
    }
    assert expected_joints.issubset(joint_names)


def test_model_declares_ros2_control_plugin() -> None:
    root = _load_xml(MODEL_PATH)
    plugins = root.findall(".//plugin")
    assert plugins, "No Gazebo plugins configured"
    control_plugins = [
        plugin
        for plugin in plugins
        if plugin.attrib.get("filename") == "libignition-gazebo-ros2-control-system.so"
    ]
    assert control_plugins, "ros2_control plugin missing"
    assert control_plugins[0].findtext("parameters") == "../config/ros2_control.yaml"


def test_ros2_control_configuration_is_consistent() -> None:
    config = json.loads(CONTROL_CONFIG_PATH.read_text(encoding="utf-8"))

    manager = config["controller_manager"]["ros__parameters"]
    assert "amiga_base_controller" in manager
    assert "kinova_arm_controller" in manager

    ros2_control = config["ros2_control"]["ros__parameters"]
    joints = ros2_control["amiga_kinova_system"]["joints"]
    for joint in (
        "left_wheel_joint",
        "right_wheel_joint",
        "kinova_joint_1",
        "kinova_joint_7",
    ):
        assert joint in joints, f"Joint {joint} missing from ros2_control configuration"

    base_params = config["amiga_base_controller"]["ros__parameters"]
    assert base_params["left_wheel_names"] == ["left_wheel_joint"]
    assert base_params["right_wheel_names"] == ["right_wheel_joint"]

    arm_joints = config["kinova_arm_controller"]["ros__parameters"]["joints"]
    assert len(arm_joints) == 7


def test_world_includes_robot_model() -> None:
    root = _load_xml(WORLD_PATH)
    include = root.find(".//include")
    assert include is not None, "World does not include any model"
    assert include.findtext("uri") == "model://amiga_kinova"


def test_launch_file_mentions_core_nodes() -> None:
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")
    for snippet in (
        "ros_gz_bridge",
        "controller_manager",
        "ros2_control_node",
        "parameter_bridge",
    ):
        assert snippet in launch_text

    assert "GZ_SIM_RESOURCE_PATH" in launch_text
