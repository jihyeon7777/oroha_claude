"""Verify a paper_export bundle the way a reader would (계측_GT_연동.md §6).

  oroha_verify_export paper_export/20260925_150000        # exit 1 when anything is wrong

Checks: MANIFEST.sha256 covers every file and every hash matches; runs.csv rows marked
included have results/<run>/{meta.yaml,events.csv,csv/summary.yaml,csv/columns.md}; included bags
have raw/<run>/bag/metadata.yaml and open with `ros2 bag info`; run ids agree between runs.csv,
results/ and the metas; calib ids used by runs exist in reproduction/records/calibration; every
tests.csv row has tests/<id>.md; README.md exists and states the time base. Writes CHECK.md when
called from oroha_paper_export.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import subprocess
import sys
import time
from pathlib import Path

import yaml


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def verify(bundle: Path) -> tuple:
    problems: list = []
    stats = {"files_listed": 0, "files_ok": 0, "runs_included": 0, "bags_checked": 0}
    man = bundle / "MANIFEST.sha256"
    if not man.exists():
        problems.append("MANIFEST.sha256 missing")
        listed = {}
    else:
        listed = {}
        for line in man.read_text().splitlines():
            if not line.strip():
                continue
            h, _, rel = line.partition("  ")
            listed[rel] = h
        stats["files_listed"] = len(listed)
        for rel, h in listed.items():
            p = bundle / rel
            if not p.exists():
                problems.append(f"manifest: {rel} missing")
            elif _sha256(p) != h:
                problems.append(f"manifest: {rel} hash mismatch")
            else:
                stats["files_ok"] += 1
        for p in bundle.rglob("*"):
            rel = p.relative_to(bundle).as_posix()
            if p.is_file() and rel not in listed and p.name not in ("MANIFEST.sha256", "CHECK.md"):
                problems.append(f"manifest: {rel} present but not listed")

    readme = bundle / "README.md"
    if not readme.exists():
        problems.append("README.md missing")
    elif "UTC" not in readme.read_text():
        problems.append("README.md does not state the time base (UTC)")

    runs_csv = bundle / "runs.csv"
    rows = list(csv.DictReader(open(runs_csv, newline=""))) if runs_csv.exists() else []
    if not runs_csv.exists():
        problems.append("runs.csv missing")
    calib_dir = bundle / "reproduction" / "records" / "calibration"
    calib_ids = set()
    for c in calib_dir.glob("*.yaml") if calib_dir.exists() else []:
        try:
            calib_ids.add(str(yaml.safe_load(open(c)).get("calib_id")))
        except Exception:  # noqa: BLE001
            pass
    included = [r for r in rows if r.get("included") == "yes"]
    stats["runs_included"] = len(included)
    for r in included:
        rid = r["run_id"]
        res = bundle / "results" / rid
        for rel in ("meta.yaml", "events.csv", "csv/summary.yaml", "csv/columns.md", "versions.yaml"):
            if not (res / rel).exists():
                problems.append(f"{rid}: results/{rid}/{rel} missing")
        meta_p = res / "meta.yaml"
        if meta_p.exists():
            meta = yaml.safe_load(open(meta_p)) or {}
            if meta.get("run_id") != rid:
                problems.append(f"{rid}: meta.yaml run_id is {meta.get('run_id')}")
            for cid in (meta.get("versions") or {}).get("calibration", {}).values():
                if str(cid) not in calib_ids:
                    problems.append(f"{rid}: calib_id {cid} not in reproduction/records/calibration")
        if r.get("bag_included") == "yes":
            bag = bundle / "raw" / rid / "bag"
            if not (bag / "metadata.yaml").exists():
                problems.append(f"{rid}: raw/{rid}/bag/metadata.yaml missing")
            else:
                stats["bags_checked"] += 1
                try:
                    out = subprocess.run(["ros2", "bag", "info", str(bag)], capture_output=True,
                                         text=True, timeout=60)
                    if out.returncode != 0 or "Duration" not in out.stdout:
                        problems.append(f"{rid}: ros2 bag info failed: {out.stderr.strip()[:120]}")
                except Exception as e:  # noqa: BLE001
                    problems.append(f"{rid}: ros2 bag info error {e}")
    for d in (bundle / "results").iterdir() if (bundle / "results").exists() else []:
        if d.is_dir() and d.name not in {r["run_id"] for r in rows}:
            problems.append(f"results/{d.name} not in runs.csv")

    tests_csv = bundle / "tests.csv"
    if tests_csv.exists():
        for r in csv.DictReader(open(tests_csv, newline="")):
            tid = r.get("test_id", "")
            if not (bundle / "tests" / f"{tid}.md").exists():
                problems.append(f"tests/{tid}.md missing")
    else:
        problems.append("tests.csv missing")
    return problems, stats


def write_check(bundle: Path, problems: list, stats: dict):
    lines = [f"# CHECK — {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())} (UTC)", "",
             f"- files listed/ok: {stats['files_listed']}/{stats['files_ok']}",
             f"- runs included: {stats['runs_included']}, bags opened: {stats['bags_checked']}",
             f"- result: {'OK' if not problems else f'{len(problems)} problem(s)'}", ""]
    lines += [f"- {p}" for p in problems]
    (bundle / "CHECK.md").write_text("\n".join(lines) + "\n")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bundle")
    ap.add_argument("--write-check", action="store_true", help="also (re)write CHECK.md in the bundle")
    a = ap.parse_args(argv)
    bundle = Path(a.bundle)
    problems, stats = verify(bundle)
    print(f"files {stats['files_ok']}/{stats['files_listed']} ok, runs {stats['runs_included']}, bags {stats['bags_checked']}")
    for p in problems:
        print("  -", p)
    print("OK" if not problems else f"{len(problems)} problem(s)")
    if a.write_check:
        write_check(bundle, problems, stats)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
