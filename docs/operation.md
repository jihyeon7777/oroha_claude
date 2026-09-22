# 운용 절차 (초안)

1. 전원: 배터리 XT60 → 모터 차단기 ON → E-stop 해제. Pi 부팅 후 `ls /dev/oroha_*`로 장치 확인.
2. `source setup/env.sh` → `oroha_preflight` (모터 무동작, RS485 포트를 독점하므로 런치 전).
3. `ros2 launch oroha_bringup robot.launch.py` (+ `imu:=true`, `rviz:=true`). `ros2 control list_controllers`로 `diff_cont` active 확인.
4. 키보드 주행: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel -p stamped:=true -p frame_id:=base_link -p speed:=0.2 -p turn:=0.5` (실험 전에는 종료 — 실행기가 다른 cmd_vel 발행자를 거부).
5. 실험: `ros2 run oroha_experiment runner`(별도 터미널) → `oroha_exp run --path square --side 1.5 --v 0.3 --repeats 3` → 조건 입력 → Enter 시작, space/Esc 중단, n 메모.
6. 추출·기록: `oroha_export_csv data/runs/<RUN_ID>` → `oroha_ledger` → 시험 기록(`records/tests/`).
7. 논문 묶음: `oroha_paper_export --runs usable` → `paper_export/<ts>/CHECK.md` 확인.

안전: 모터를 도는 첫 시험은 바퀴를 띄운 상태, E-stop 손에, 접지 시 고임목·감시자. 호스트가 멈추면 물리 E-stop만 로봇을 세운다.
