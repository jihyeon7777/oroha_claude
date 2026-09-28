"""Ledger rules and paper-bundle verification (no ROS needed)."""

import csv
import hashlib

import yaml

from oroha_tools import ledger, verify_export


def _meta(tmp, run_id, status, rig_state):
    d = tmp / run_id
    d.mkdir(parents=True)
    (d / "meta.yaml").write_text(yaml.safe_dump({"run_id": run_id, "status": status,
                                                 "rig_state": rig_state,
                                                 "conditions": {"rig_state": rig_state}}))


def test_usable_requires_done_on_ground(tmp_path):
    _meta(tmp_path, "R1-ground", "DONE", "on_ground")
    _meta(tmp_path, "R2-mock", "DONE", "mock")
    _meta(tmp_path, "R3-lifted", "DONE", "lifted")
    _meta(tmp_path, "R4-abort", "ABORTED", "on_ground")
    rows = {r["run_id"]: r for r in ledger.refresh_runs(tmp_path)}
    assert rows["R1-ground"]["usable"] == "yes"
    for rid in ("R2-mock", "R3-lifted", "R4-abort"):
        assert rows[rid]["usable"] == "no" and rows[rid]["exclude_reason"]


def test_manual_exclusion_survives_refresh(tmp_path):
    _meta(tmp_path, "R1-ground", "DONE", "on_ground")
    ledger.refresh_runs(tmp_path, ("R1-ground", "wheel slipped on cable"))
    rows = {r["run_id"]: r for r in ledger.refresh_runs(tmp_path)}
    assert rows["R1-ground"]["usable"] == "no"
    assert rows["R1-ground"]["exclude_reason"] == "wheel slipped on cable"


def _bundle(tmp_path):
    b = tmp_path / "bundle"
    (b / "tests").mkdir(parents=True)
    (b / "README.md").write_text("# bundle\ntimes are UTC\n")
    (b / "tests.csv").write_text("test_id,status\nT1,통과\n")
    (b / "tests" / "T1.md").write_text("# T1\n")
    with open(b / "runs.csv", "w", newline="") as f:
        csv.writer(f).writerows([["run_id", "included", "bag_included"]])
    lines = []
    for p in sorted(b.rglob("*")):
        if p.is_file():
            lines.append(f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(b).as_posix()}")
    (b / "MANIFEST.sha256").write_text("\n".join(lines) + "\n")
    return b


def test_verify_export_ok_and_detects_missing_and_changed(tmp_path):
    b = _bundle(tmp_path)
    problems, stats = verify_export.verify(b)
    assert problems == [] and stats["files_ok"] == stats["files_listed"] == 4
    (b / "tests" / "T1.md").write_text("# edited\n")
    (b / "tests.csv").unlink()
    problems, _ = verify_export.verify(b)
    joined = "\n".join(problems)
    assert "tests/T1.md hash mismatch" in joined
    assert "tests.csv missing" in joined
