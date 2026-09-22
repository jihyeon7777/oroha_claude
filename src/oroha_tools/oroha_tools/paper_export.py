"""Build a self-contained evidence bundle for the paper: paper_export/<YYYYMMDD_HHMMSS>/.

  oroha_paper_export --runs usable --out paper_export/$(date +%Y%m%d_%H%M%S)
  oroha_paper_export --runs R20260925-143012-square,R20260925-144001-circle --no-bags

Bundle layout (계측_GT_연동.md §5):
  README.md            scope, timezone, constants, file map, exclusions, how to verify
  tests.csv, tests/    the test ledger and per-test notes (copied verbatim)
  runs.csv             every run with usable / exclude_reason (also the excluded ones)
  reproduction/        oroha.repos, configs, calibration YAMLs, firmware, setup scripts, patches, changes/decisions
  results/<run_id>/    meta, events, profile, conditions, versions, uncommitted diff, csv/ (exported if missing)
  raw/<run_id>/bag/    the rosbag (all files incl. metadata.yaml) unless --no-bags (then listed as 별도보관)
  MANIFEST.sha256      bundle-relative, `sha256sum -c MANIFEST.sha256` from inside the bundle
  CHECK.md             result of oroha_verify_export on the fresh bundle
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

from oroha_tools.ws import records_dir, runs_dir, workspace_root

REPRO_FILES = [
    "oroha.repos", "README.md", "CLAUDE.md", "개발_개요.md", "하드웨어_확인표.md", "계측_GT_연동.md",
    "src/oroha_bringup/config/oroha_controllers.yaml", "src/oroha_bringup/config/power.yaml",
    "src/oroha_bringup/config/um7.yaml", "src/oroha_description/urdf/oroha.urdf.xacro",
    "src/oroha_description/urdf/oroha.ros2_control.xacro", "firmware/pico/main.py",
    "firmware/pico/main.py.sha256", "firmware/pico/README.md", "records/changes.md",
    "records/decisions.md",
]
REPRO_DIRS = ["setup", "patches", "records/calibration", "records/preflight", "firmware/pico/archive"]


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def _copy(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)
    if src.is_dir():
        shutil.copytree(src, dst, dirs_exist_ok=True, symlinks=False)
    else:
        shutil.copy2(src, dst, follow_symlinks=True)


def select_runs(spec: str, runs_csv: Path) -> tuple:
    """(selected run ids, all rows)"""
    rows = list(csv.DictReader(open(runs_csv, newline="")))
    if spec == "all":
        sel = [r["run_id"] for r in rows]
    elif spec == "usable":
        sel = [r["run_id"] for r in rows if r.get("usable") == "yes"]
    else:
        sel = [s.strip() for s in spec.split(",") if s.strip()]
    return sel, rows


def build(out: Path, runs_spec: str, include_bags: bool, ws: Path) -> dict:
    from oroha_tools.export_csv import export as export_csv
    from oroha_tools.ledger import refresh_runs

    refresh_runs(runs_dir())
    runs_csv = runs_dir() / "runs.csv"
    selected, all_rows = select_runs(runs_spec, runs_csv)
    out.mkdir(parents=True, exist_ok=False)
    tz = time.strftime("%Z")
    stamp_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    # ledger + tests
    _copy(records_dir() / "tests" / "tests.csv", out / "tests.csv")
    for md in (records_dir() / "tests").glob("*.md"):
        _copy(md, out / "tests" / md.name)
    with open(out / "runs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()) + ["included", "bag_included"] if all_rows else ["run_id"])
        w.writeheader()
        for r in all_rows:
            r = dict(r)
            r["included"] = "yes" if r["run_id"] in selected else "no"
            r["bag_included"] = "yes" if (r["run_id"] in selected and include_bags) else "no"
            w.writerow(r)

    # reproduction material
    for rel in REPRO_FILES:
        p = ws / rel
        if p.exists():
            _copy(p, out / "reproduction" / rel)
    for rel in REPRO_DIRS:
        p = ws / rel
        if p.exists():
            _copy(p, out / "reproduction" / rel)
    try:
        ext = subprocess.run(["vcs", "export", "--exact", str(ws / "src" / "external")],
                             capture_output=True, text=True, timeout=30).stdout
        (out / "reproduction" / "external_versions.repos").write_text(ext)
    except Exception:  # noqa: BLE001
        pass

    # results + raw
    missing = []
    for rid in selected:
        run = runs_dir() / rid
        if not (run / "meta.yaml").exists():
            missing.append(rid)
            continue
        if not (run / "csv" / "summary.yaml").exists() and (run / "bag" / "metadata.yaml").exists():
            try:
                export_csv(run, run / "csv")
            except Exception as e:  # noqa: BLE001
                print(f"  export failed for {rid}: {e}")
        for item in run.iterdir():
            if item.name == "bag":
                continue
            _copy(item, out / "results" / rid / item.name)
        if include_bags and (run / "bag").exists():
            _copy(run / "bag", out / "raw" / rid / "bag")

    # README
    excluded = [(r["run_id"], r.get("exclude_reason", "")) for r in all_rows if r["run_id"] not in selected]
    summary_src = records_dir() / "paper_summary.md"
    summary = summary_src.read_text() if summary_src.exists() else "_records/paper_summary.md 없음 — 요약은 아직 작성되지 않았다._\n"
    calib_lines = []
    for c in sorted((records_dir() / "calibration").glob("*.yaml")):
        try:
            d = yaml.safe_load(open(c))
            calib_lines.append(f"| `{d.get('calib_id', c.stem)}` | `reproduction/records/calibration/{c.name}` | {d.get('status', d.get('uncertainty', ''))} |")
        except Exception:  # noqa: BLE001
            pass
    readme = f"""# OROHA 논문 근거자료 묶음 — {out.name}

생성 {stamp_utc} (UTC) · 생성 시각대 {tz} · 워크스페이스 `{ws}` · 실행 선택 `{runs_spec}` · bag 포함 {'예' if include_bags else '아니오(별도보관)'}

**시간 기준**: 모든 `*_utc`는 UTC. bag·CSV의 `ros_t_ns`/`stamp_ns`는 Pi 시스템 시계(chrony)의 ns epoch. 각 실행의 `versions.yaml`에 당시 시간대와 chrony 상태가 있다. CSV 열·단위는 각 `results/<run>/csv/columns.md`.

## 논문용 요약

{summary}

## 확정 상수

| calib_id | 파일 | 상태 |
|---|---|---|
{chr(10).join(calib_lines)}

## 파일 지도

| 경로 | 내용 |
|---|---|
| `tests.csv`, `tests/*.md` | 개발·시험 목록과 시험별 목적·조건·절차·결과·판정·로그 |
| `runs.csv` | 모든 실험 실행(포함 여부 `included`, 제외 사유 `exclude_reason`, bag 포함 `bag_included`) |
| `results/<run_id>/` | meta.yaml(조건·버전·상태) · events.csv · profile.csv(명령·이상 경로) · conditions.yaml · versions.yaml · uncommitted.diff · csv/(추출본, summary.yaml·columns.md) |
| `raw/<run_id>/bag/` | rosbag2(mcap + metadata.yaml) — 원자료 |
| `reproduction/` | 저장소 문서·설정·교정 YAML·펌웨어(+sha256)·setup 스크립트·패치·변경/결정 기록·외부 소스 정확 버전(`external_versions.repos`) |
| `MANIFEST.sha256` | 무결성: 묶음 안에서 `sha256sum -c MANIFEST.sha256` |
| `CHECK.md` | 생성 직후 `oroha_verify_export` 결과 |

## 제외된 실행 ({len(excluded)}건)

{chr(10).join(f'- `{rid}` — {reason or "(사유 없음)"}' for rid, reason in excluded) or '- 없음'}

## 누락 ({len(missing)}건)

{chr(10).join(f'- `{rid}` — meta.yaml 없음' for rid in missing) or '- 없음'}

## 재현

`reproduction/README.md`의 빠른 시작을 따른다: `oroha.repos`의 커밋으로 외부 소스를 받고, `reproduction/src/...` 설정과 `reproduction/records/calibration/`의 교정값, `reproduction/firmware/pico/main.py`(sha256 동봉)를 쓴다. 실행마다 `results/<run>/versions.yaml`이 당시 커밋·미커밋 diff(`uncommitted.diff`)·펌웨어 sha·교정 ID를 준다.
"""
    (out / "README.md").write_text(readme)

    # manifest (relative to the bundle root, excluding itself and CHECK.md which is written after)
    entries = []
    for p in sorted(out.rglob("*")):
        if p.is_file() and p.name not in ("MANIFEST.sha256", "CHECK.md"):
            entries.append(f"{_sha256(p)}  {p.relative_to(out).as_posix()}")
    (out / "MANIFEST.sha256").write_text("\n".join(entries) + "\n")
    return {"out": str(out), "runs": selected, "excluded": excluded, "missing": missing,
            "files": len(entries), "bags": include_bags}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", default="usable", help="usable | all | comma-separated run ids")
    ap.add_argument("--out", default=None, help="output dir (default paper_export/<YYYYMMDD_HHMMSS>)")
    ap.add_argument("--no-bags", action="store_true", help="do not copy rosbags (listed as 별도보관)")
    ap.add_argument("--no-check", action="store_true", help="skip the verification pass")
    a = ap.parse_args(argv)
    ws = workspace_root()
    out = Path(a.out) if a.out else ws / "paper_export" / time.strftime("%Y%m%d_%H%M%S")
    info = build(out, a.runs, not a.no_bags, ws)
    print(f"bundle {info['out']}: {len(info['runs'])} runs, {info['files']} files, bags={'yes' if info['bags'] else 'no'}")
    if info["missing"]:
        print("  missing runs:", info["missing"])
    if not a.no_check:
        from oroha_tools.verify_export import verify, write_check
        problems, stats = verify(out)
        write_check(out, problems, stats)
        print("verify: %s" % ("OK" if not problems else f"{len(problems)} problem(s), see CHECK.md"))
        return 1 if problems else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
