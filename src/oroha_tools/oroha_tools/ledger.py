"""Refresh data/runs/runs.csv from every data/runs/*/meta.yaml and sanity-check records/tests/tests.csv.

  oroha_ledger            # rewrite runs.csv, report counts
  oroha_ledger --check    # also validate tests.csv (ids unique, vocab, referenced files exist)

runs.csv keeps the exclusion state of every run (`usable`, `exclude_reason`); edit those two
columns by hand (or with --exclude RUN_ID "reason") — everything else is regenerated.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import yaml

from oroha_tools.ws import records_dir, runs_dir

RUN_COLUMNS = ["run_id", "series_id", "repeat_index", "path", "params", "status", "armed_utc",
               "start_utc", "end_utc", "t_run_end", "timezone", "rig_state", "surface",
               "floor_slope_deg", "robot_mass_kg", "payload_kg", "tyre_pressure_kpa", "ambient_c",
               "ws_describe", "ws_dirty", "fw_sha256", "calib_ids", "bag_bytes", "csv_exported",
               "test_id", "usable", "exclude_reason", "note"]
STATUS_VOCAB = {"통과", "실패", "중단", "미검증"}
VERDICT_VOCAB = {"종결", "열림", "다시 열림", "재현 실패", "미수렴"}


def _rig_state(meta: dict) -> str:
    return str(meta.get("rig_state") or (meta.get("conditions") or {}).get("rig_state") or "")


def default_usable(meta: dict) -> bool:
    """Only completed runs on the ground (real hardware) are paper data by default."""
    return meta.get("status") == "DONE" and _rig_state(meta) == "on_ground"


def default_reason(meta: dict) -> str:
    if default_usable(meta):
        return ""
    return "status %s, rig_state %s" % (meta.get("status", "?"), _rig_state(meta) or "unset")


def load_existing(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path, newline="") as f:
        return {r["run_id"]: r for r in csv.DictReader(f)}


def run_row(run: Path, old: dict) -> dict:
    meta = yaml.safe_load(open(run / "meta.yaml")) or {}
    cond = meta.get("conditions") or {}
    ver = meta.get("versions") or {}
    ws = ver.get("workspace") or {}
    calib = ver.get("calibration") or {}
    row = {
        "run_id": meta.get("run_id", run.name), "series_id": meta.get("series_id", ""),
        "repeat_index": meta.get("repeat_index", ""), "path": meta.get("path", ""),
        "params": yaml.safe_dump(meta.get("params", {}), default_flow_style=True, allow_unicode=True).strip(),
        "status": meta.get("status", ""), "armed_utc": meta.get("armed_utc", ""),
        "start_utc": meta.get("start_utc", ""), "end_utc": meta.get("end_utc", ""),
        "t_run_end": meta.get("t_run_end", ""), "timezone": meta.get("timezone", ""),
        "rig_state": cond.get("rig_state", ""), "surface": cond.get("surface", ""),
        "floor_slope_deg": cond.get("floor_slope_deg", ""), "robot_mass_kg": cond.get("robot_mass_kg", ""),
        "payload_kg": cond.get("payload_kg", ""), "tyre_pressure_kpa": cond.get("tyre_pressure_kpa", ""),
        "ambient_c": cond.get("ambient_c", ""),
        "ws_describe": ws.get("describe", ""), "ws_dirty": ws.get("dirty", ""),
        "fw_sha256": (ver.get("firmware_sha256") or "")[:12],
        "calib_ids": ";".join(sorted(set(str(v) for v in calib.values()))),
        "bag_bytes": (meta.get("bag") or {}).get("bytes", ""),
        "csv_exported": (run / "csv" / "summary.yaml").exists(),
        "test_id": old.get("test_id", ""),
        "usable": old.get("usable", "yes" if default_usable(meta) else "no"),
        "exclude_reason": old.get("exclude_reason", default_reason(meta)),
        "note": old.get("note", ""),
    }
    return row


def refresh_runs(runs: Path, exclude: tuple | None = None) -> list:
    out = runs / "runs.csv"
    existing = load_existing(out)
    rows = []
    for run in sorted(p for p in runs.glob("R*") if (p / "meta.yaml").exists()):
        rows.append(run_row(run, existing.get(run.name, {})))
    if exclude:
        rid, reason = exclude
        for r in rows:
            if r["run_id"] == rid:
                r["usable"], r["exclude_reason"] = "no", reason
    runs.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=RUN_COLUMNS)
        w.writeheader()
        w.writerows(rows)
    return rows


def check_tests(records: Path) -> list:
    problems = []
    path = records / "tests" / "tests.csv"
    if not path.exists():
        return ["tests.csv missing"]
    seen = set()
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            tid = r.get("test_id", "")
            if tid in seen:
                problems.append(f"duplicate test_id {tid}")
            seen.add(tid)
            if r.get("status") not in STATUS_VOCAB:
                problems.append(f"{tid}: status '{r.get('status')}' not in {sorted(STATUS_VOCAB)}")
            if r.get("verdict") not in VERDICT_VOCAB:
                problems.append(f"{tid}: verdict '{r.get('verdict')}' not in {sorted(VERDICT_VOCAB)}")
            if not (records / "tests" / f"{tid}.md").exists():
                problems.append(f"{tid}: records/tests/{tid}.md missing")
            for ref in (r.get("retest_of", ""), r.get("superseded_by", "")):
                if ref and ref not in seen and not (records / "tests" / f"{ref}.md").exists():
                    problems.append(f"{tid}: reference {ref} unknown")
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="validate records/tests/tests.csv")
    ap.add_argument("--exclude", nargs=2, metavar=("RUN_ID", "REASON"), help="mark a run unusable")
    a = ap.parse_args(argv)
    rows = refresh_runs(runs_dir(), tuple(a.exclude) if a.exclude else None)
    usable = sum(1 for r in rows if r["usable"] == "yes")
    print(f"runs.csv: {len(rows)} runs, {usable} usable -> {runs_dir() / 'runs.csv'}")
    rc = 0
    if a.check:
        probs = check_tests(records_dir())
        for p in probs:
            print("  tests.csv:", p)
        print("tests.csv: %s" % ("OK" if not probs else f"{len(probs)} problem(s)"))
        rc = 1 if probs else 0
    return rc


if __name__ == "__main__":
    sys.exit(main())
