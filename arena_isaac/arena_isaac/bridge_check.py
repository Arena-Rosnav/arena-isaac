"""Isaac start check: the bridge message types load from their own prefix and rclpy, rcl and Fast DDS match the arena side."""

from __future__ import annotations

import importlib
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections.abc import Iterable
from pathlib import Path

PACKAGES = ("isaacsim_msgs", "arena_people_msgs")
VERSIONS_ENV = "ARENA_ROS_VERSIONS"
_FASTDDS_LIB = re.compile(r"^libfastrtps\.so\.(\d+\.\d+\.\d+)$")
_INTERFACES = Path("share", "ament_index", "resource_index", "rosidl_interfaces")


def prefixes() -> list[Path]:
    """The ament prefixes of this environment, first one wins."""
    return [Path(p) for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]


def ros_versions(search: Iterable[Path]) -> dict[str, str]:
    """rclpy, rcl and Fast DDS versions from the first prefix that has each, a missing one is left out."""
    found: dict[str, str] = {}
    for prefix in search:
        for name in ("rclpy", "rcl"):
            xml = prefix / "share" / name / "package.xml"
            if name not in found and xml.is_file():
                found[name] = (ET.parse(xml).findtext("version") or "").strip()
        lib = prefix / "lib"
        if "fastdds" not in found and lib.is_dir():
            for entry in sorted(lib.iterdir()):
                if match := _FASTDDS_LIB.match(entry.name):
                    found["fastdds"] = match.group(1)
    return found


def version_drift(ours: dict[str, str], theirs: dict[str, str]) -> list[str]:
    """One line per component whose version differs between the two sides or is missing on either."""
    return [f"{name}: isaac {ours.get(name, 'missing')}, arena {theirs.get(name, 'missing')}" for name in ("rclpy", "rcl", "fastdds") if ours.get(name) is None or ours.get(name) != theirs.get(name)]


def type_failures(search: Iterable[Path], packages: Iterable[str] = PACKAGES) -> list[str]:
    """One line per bridge type that does not import, load its type support and round-trip serialization from its own prefix."""
    from rclpy.serialization import deserialize_message, serialize_message
    from rclpy.type_support import check_for_type_support

    search = list(search)
    failures: list[str] = []
    for package in packages:
        prefix = next((p for p in search if (p / _INTERFACES / package).is_file()), None)
        if prefix is None:
            failures.append(f"{package}: not on AMENT_PREFIX_PATH")
            continue
        module = importlib.import_module(package)
        if not Path(module.__file__ or "").is_relative_to(prefix):
            failures.append(f"{package}: imported from {module.__file__}, not from {prefix}")
        for entry in (prefix / _INTERFACES / package).read_text().split():
            kind, _, file = entry.partition("/")
            if kind not in ("msg", "srv") or not file.endswith(f".{kind}"):
                continue
            name = f"{package}/{kind}/{file.removesuffix(f'.{kind}')}"
            try:
                cls = getattr(importlib.import_module(f"{package}.{kind}"), file.removesuffix(f".{kind}"))
                check_for_type_support(cls)
                for sub in (cls.Request, cls.Response) if kind == "srv" else (cls,):
                    deserialize_message(serialize_message(sub()), sub)
            except Exception as e:
                failures.append(f"{name}: {type(e).__name__}: {e}")
    return failures


def main(argv: list[str]) -> int:
    if argv == ["versions"]:
        print(json.dumps(ros_versions(prefixes())))
        return 0
    if argv != ["check"]:
        print("usage: bridge_check.py versions | check (check compares against the JSON in $ARENA_ROS_VERSIONS)", file=sys.stderr)
        return 2
    problems = type_failures(prefixes())
    raw = os.environ.get(VERSIONS_ENV)
    expected: dict[str, str] = {} if raw is None else json.loads(raw)
    if raw is None:
        problems.append(f"{VERSIONS_ENV} is not set, the arena side versions are unknown")
    else:
        problems += version_drift(ros_versions(prefixes()), expected)
    if problems:
        print("isaac bridge check failed, Isaac Sim is not started:", file=sys.stderr)
        for line in problems:
            print(f"  {line}", file=sys.stderr)
        return 1
    print(f"isaac bridge check: {', '.join(PACKAGES)} load, {expected} match")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
