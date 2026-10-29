"""The isaac start check: version reading and drift, type loading from the bridge prefix, and the check verb's exit status."""

import importlib.util
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from arena_isaac import bridge_check

needs_msgs = pytest.mark.skipif(importlib.util.find_spec("rclpy") is None or importlib.util.find_spec("isaacsim_msgs") is None, reason="needs rclpy and a built isaacsim_msgs")
SCRIPT = Path(bridge_check.__file__)


def _package(prefix: Path, name: str, version: str) -> None:
    (prefix / "share" / name).mkdir(parents=True)
    (prefix / "share" / name / "package.xml").write_text(f'<?xml version="1.0"?>\n<package format="3"><name>{name}</name><version>{version}</version></package>\n')


def test_versions_come_from_the_first_prefix_that_has_each(tmp_path: Path) -> None:
    first, second = tmp_path / "first", tmp_path / "second"
    _package(first, "rclpy", "7.1.11")
    _package(second, "rclpy", "7.0.0")
    _package(second, "rcl", "9.2.11")
    (second / "lib").mkdir()
    for name in ("libfastrtps.so", "libfastrtps.so.2.14", "libfastrtps.so.2.14.6"):
        (second / "lib" / name).write_bytes(b"")
    assert bridge_check.ros_versions([first, second]) == {"rclpy": "7.1.11", "rcl": "9.2.11", "fastdds": "2.14.6"}


def test_drift_names_every_differing_or_missing_component() -> None:
    arena = {"rclpy": "7.1.11", "rcl": "9.2.11", "fastdds": "2.14.6"}
    assert bridge_check.version_drift(dict(arena), arena) == []
    assert bridge_check.version_drift({"rclpy": "7.1.11", "rcl": "9.2.12"}, arena) == ["rcl: isaac 9.2.12, arena 9.2.11", "fastdds: isaac missing, arena 2.14.6"]


@needs_msgs
def test_every_installed_bridge_type_loads_from_its_prefix() -> None:
    assert bridge_check.type_failures(bridge_check.prefixes()) == []


@needs_msgs
def test_a_package_missing_from_the_prefixes_is_reported() -> None:
    assert bridge_check.type_failures([], ["isaacsim_msgs"]) == ["isaacsim_msgs: not on AMENT_PREFIX_PATH"]


@needs_msgs
def test_a_module_imported_from_another_prefix_is_reported(tmp_path: Path) -> None:
    import isaacsim_msgs

    index = Path("share", "ament_index", "resource_index", "rosidl_interfaces", "isaacsim_msgs")
    installed = next(p / index for p in bridge_check.prefixes() if (p / index).is_file())
    (tmp_path / index).parent.mkdir(parents=True)
    shutil.copy(installed, tmp_path / index)
    failures = bridge_check.type_failures([tmp_path], ["isaacsim_msgs"])
    assert failures == [f"isaacsim_msgs: imported from {isaacsim_msgs.__file__}, not from {tmp_path}"]


def _check(versions: dict[str, str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run([sys.executable, str(SCRIPT), "check"], env={**os.environ, bridge_check.VERSIONS_ENV: json.dumps(versions)}, capture_output=True, text=True, check=False)


@needs_msgs
def test_check_passes_against_this_sides_own_versions() -> None:
    done = _check(bridge_check.ros_versions(bridge_check.prefixes()))
    assert done.returncode == 0, done.stderr


@needs_msgs
def test_check_fails_on_a_drifted_version() -> None:
    versions = bridge_check.ros_versions(bridge_check.prefixes())
    done = _check({**versions, "rcl": "0.0.0"})
    assert done.returncode == 1
    assert f"rcl: isaac {versions['rcl']}, arena 0.0.0" in done.stderr
