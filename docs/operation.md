# 운용 절차

## 안전 원칙 (2026-09-28 재검토)

- MD400에는 통신 워치독이 없다. 호스트가 명령을 못 보내면(런치 Ctrl-C·`ros2_control_node` 크래시·RS485 분리) **마지막 속도 명령이 남는다.**
  - 런치 안전망: `ros2_control_node`가 끝나면 launch가 양쪽 MD400에 VEL_CMD 0·stop·torque_off를 보낸다(실물, `safety_stop:=true` 기본).
  - 수동: `oroha_md_stop`. E-stop 중에도 로직 전원은 살아 있어 동작한다.
  - RS485가 빠지면 아무것도 전달되지 않는다 → **E-stop**. 재연결 후 `oroha_md_stop`으로 0 확인, 그다음 해제.
- **모든 모션 명령은 스스로 끝나야 한다**: `ros2 topic pub -r 10 -t 30 …`(3 s), 시간 제한 프로파일, `oroha_direction_check --sec N`. `-r` 단독 무기한 발행, 타임아웃 없는 컨트롤러(forward_command_controller) 금지.
- E-stop은 사용자 손에. Claude의 도구 호출은 실시간 정지 경로가 아니다(지연·거부될 수 있음).
- **E-stop 해제 규칙**: E1 시험(세션 A, `oroha_direction_check --sec 10`, 도는 중 누름→해제)으로 확정 예정. 확정 전에는 해제 전에 명령이 0인지 확인한다(런치 종료 또는 `oroha_md_stop`).
- 접지 주행은 3 m×3 m 방에서, `oroha_exp`가 계산한 필요 공간(궤적 + 차체 반대각 + 여유 0.3 m)을 통과한 경로만, 사용자 터미널에서 1회씩 시작한다.

## 순서

1. 전원: 배터리 XT60 → 모터 차단기 ON → E-stop 해제(명령 0 상태). Pi 부팅 후 `ls /dev/oroha_*`, `timedatectl`(NTP 동기).
2. `source setup/env.sh` → `oroha_preflight` (모터 무동작, 포트를 연 다른 프로세스가 있으면 거부 — 런치 전에).
3. `ros2 launch oroha_bringup robot.launch.py` (+ `imu:=true`, `rviz:=true`). `ros2 control list_controllers`로 `diff_cont` active 확인.
4. 키보드 주행(사용자 터미널): `ros2 run oroha_teleop deadman_teleop` — 누르고 있는 동안만 주행, space/Esc 즉시 정지. 실험 전에는 종료(실행기가 다른 cmd_vel 발행자를 거부).
5. 실험: `ros2 run oroha_experiment runner`(별도 터미널) → `oroha_exp run --path square --side 1.0 --v 0.2 --rig-state on_ground` → 조건 확인 → 안내된 시작 위치에 로봇 → Enter 시작, space/Esc/Ctrl-C 중단, n 메모 → 종점 줄자 측정 → `oroha_exp gt --run <RUN_ID> --x … --y …`.
6. 추출·기록: `oroha_export_csv data/runs/<RUN_ID>` → `oroha_ledger --check` → 시험 기록(`records/tests/`).
7. 논문 묶음: `oroha_paper_export --runs usable` → `paper_export/<ts>/CHECK.md` 확인.

## 통신 재연결 (개요 §4)

`mdrobot_ros2_control`은 5회 연속 통신 실패 시 하드웨어 컴포넌트를 ERROR로 내리고 torque_off를 시도한다. 복구: 원인 제거(케이블) → `oroha_md_stop` → `ros2 control set_hardware_component_state oroha_base inactive` → `… active` → `ros2 control switch_controllers --activate diff_cont` (절차는 세션 A 정지 시험에서 검증 후 확정).
