"""Snapshot everything a run or test must be reproducible from.

  oroha_versions                  # print YAML
  oroha_versions --out <dir>      # write versions.yaml + uncommitted.diff + untracked.txt (+ copies)

Captures: workspace git (describe --dirty, HEAD, branch), external repo SHAs and
dirty flags (src/external via `vcs export --exact` style git calls), applied
patches, Pico firmware file sha256, calibration ids, resolved config hashes, ROS
package versions, OS/kernel, timezone, chrony tracking. Uncommitted changes are
saved as a diff plus copies of untracked files under src/ and records/ so a run
can be rebuilt even from a dirty tree (계측_GT_연동.md §4).
"""

from __future__ import annotations

import argparse
import hashlib
import os
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

from oroha_tools.ws import workspace_root

CONFIG_FILES = [
    "src/oroha_bringup/config/oroha_controllers.yaml",
    "src/oroha_bringup/config/power.yaml",
    "src/oroha_bringup/config/um7.yaml",
    "src/oroha_description/urdf/oroha.urdf.xacro",
    "src/oroha_description/urdf/oroha.ros2_control.xacro",
]
ROS_PKGS = ["ros2_control", "ros2_controllers", "diff_drive_controller", "hardware_interface",
            "controller_manager", "rosbag2", "xacro"]


def _run(cmd, cwd=None, timeout=20) -> str:
    try:
        return subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=timeout,
                              check=False).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def _git_info(repo: Path) -> dict:
    if not (repo / ".git").exists():
        return {"error": "not a git repo"}
    return {
        "head": _run(["git", "rev-parse", "HEAD"], repo),
        "describe": _run(["git", "describe", "--tags", "--dirty", "--always"], repo),
        "branch": _run(["git", "rev-parse", "--abbrev-ref", "HEAD"], repo),
        "dirty": bool(_run(["git", "status", "--porcelain"], repo)),
    }


def ros_pkg_versions() -> dict:
    out = {}
    for p in ROS_PKGS:
        xml = _run(["ros2", "pkg", "xml", p, "-t", "version"])
        if xml:
            out[p] = xml
    return out


def snapshot(ws: Path) -> dict:
    ext = {}
    ext_dir = ws / "src" / "external"
    if ext_dir.exists():
        for repo in sorted(ext_dir.iterdir()):
            if repo.is_dir():
                ext[repo.name] = _git_info(repo)
    patches = sorted(str(p.relative_to(ws)) for p in (ws / "patches").rglob("*.patch"))
    fw = ws / "firmware" / "pico" / "main.py"
    calib = {}
    for c in sorted((ws / "records" / "calibration").glob("*.yaml")):
        try:
            with open(c) as f:
                calib[c.name] = yaml.safe_load(f).get("calib_id", "?")
        except Exception:  # noqa: BLE001
            calib[c.name] = "unreadable"
    configs = {}
    for rel in CONFIG_FILES:
        p = ws / rel
        if p.exists():
            configs[rel] = _sha256(p)
    return {
        "captured_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "captured_local": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
        "timezone": _run(["timedatectl", "show", "-p", "Timezone", "--value"]) or time.tzname[0],
        "chrony_tracking": _run(["chronyc", "tracking"]).splitlines()[:6],
        "host": {"hostname": platform.node(), "os": platform.platform(),
                 "kernel": platform.release(), "python": platform.python_version()},
        "workspace": {"path": str(ws), **_git_info(ws)},
        "external": ext,
        "patches_applied": patches,
        "firmware": {"file": "firmware/pico/main.py",
                     "sha256": _sha256(fw) if fw.exists() else None},
        "calibration": calib,
        "config_sha256": configs,
        "ros_packages": ros_pkg_versions(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "rmw": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp (default)"),
    }


def write_snapshot(ws: Path, out: Path) -> dict:
    """versions.yaml + uncommitted.diff + untracked.txt + untracked/ copies into `out`."""
    out.mkdir(parents=True, exist_ok=True)
    snap = snapshot(ws)
    with open(out / "versions.yaml", "w") as f:
        yaml.safe_dump(snap, f, allow_unicode=True, sort_keys=False)
    diff = _run(["git", "diff", "HEAD", "--", "src", "records", "setup", "firmware",
                 "oroha.repos", "patches"], ws, timeout=60)
    (out / "uncommitted.diff").write_text(diff + ("\n" if diff else ""))
    untracked = _run(["git", "ls-files", "--others", "--exclude-standard", "src", "records",
                      "setup", "firmware", "patches"], ws, timeout=60).splitlines()
    (out / "untracked.txt").write_text("\n".join(untracked) + ("\n" if untracked else ""))
    if untracked:
        dst_root = out / "untracked"
        for rel in untracked:
            src = ws / rel
            if src.is_file():
                dst = dst_root / rel
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)
    snap["_uncommitted_diff_bytes"] = len(diff)
    snap["_untracked_files"] = len(untracked)
    return snap


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", help="directory to write versions.yaml, uncommitted.diff, untracked.txt")
    a = ap.parse_args(argv)
    ws = workspace_root()
    if a.out:
        snap = write_snapshot(ws, Path(a.out))
        print("wrote %s (diff %d bytes, %d untracked files)" % (
            a.out, snap["_uncommitted_diff_bytes"], snap["_untracked_files"]))
    else:
        yaml.safe_dump(snapshot(ws), sys.stdout, allow_unicode=True, sort_keys=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
