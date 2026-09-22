# 시험 기록

- `tests.csv` — 시험 목록. `status` ∈ 통과/실패/중단/미검증, `verdict` ∈ 종결/열림/다시 열림/재현 실패/미수렴. `rig_state` ∈ bench(무동작)/lifted(바퀴 띄움)/on_ground(접지)/mock.
- `T<YYYYMMDD>-<NN>.md` — 시험별 목적·조건·절차·관측·판정 근거·로그 위치·버전.
- 실행(run) 데이터는 `data/runs/<RUN_ID>/`; 시험이 실행을 참조하면 `log_paths`에 그 경로를 적는다.
