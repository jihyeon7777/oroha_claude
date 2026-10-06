# 운용 절차

## 안전 원칙 (2026-09-28 재검토)

- MD400에는 통신 워치독이 없다. 호스트가 명령을 못 보내면(런치 Ctrl-C·`ros2_control_node` 크래시·RS485 분리) **마지막 속도 명령이 남는다.**
  - 런치 안전망: `ros2_control_node`가 끝나면 launch가 양쪽 MD400에 VEL_CMD 0·stop·torque_off를 보낸다(실물, `safety_stop:=true` 기본).
  - 수동: `oroha_md_stop`. E-stop 중에도 로직 전원은 살아 있어 동작한다.
  - RS485가 빠지면 아무것도 전달되지 않는다 → **E-stop**. 재연결 후 `oroha_md_stop`으로 0 확인, 그다음 해제.
- **모든 모션 명령은 스스로 끝나야 한다**: `ros2 topic pub -r 10 -t 30 …`(3 s), 시간 제한 프로파일, `oroha_direction_check --sec N`. `-r` 단독 무기한 발행, 타임아웃 없는 컨트롤러(forward_command_controller) 금지.
- E-stop은 사용자 손에. Claude의 도구 호출은 실시간 정지 경로가 아니다(지연·거부될 수 있음).
- **E-stop**: `USE_LIMIT_SW 1`(설정값)에서만 동작 — 누르면 0.13~0.14 s에 정지(코스팅). **해제 규칙: 명령 소스(텔레옵·실행기)를 먼저 멈추고 `oroha_md_stop` 또는 런치 종료로 명령 0을 확인한 뒤 해제한다** — 명령이 계속 오면 해제 즉시 다시 돈다(T20260929-01).
- 접지 주행은 3 m×3 m 방에서, `oroha_exp`가 계산한 필요 공간(궤적 + 차체 반대각 + 여유 0.3 m)을 통과한 경로만, 사용자 터미널에서 1회씩 시작한다.

## 순서

1. 전원: 배터리 XT60 → 모터 차단기 ON → E-stop 해제(명령 0 상태). Pi 부팅 후 `ls /dev/oroha_*`, `timedatectl`(NTP 동기).
2. `source setup/env.sh` → `oroha_preflight` (모터 무동작, 포트를 연 다른 프로세스가 있으면 거부 — 런치 전에).
3. `ros2 launch oroha_bringup robot.launch.py` (+ `imu:=true`, `rviz:=true`). `ros2 control list_controllers`로 `diff_cont` active 확인.
4. 키보드 주행(사용자 터미널): `ros2 run oroha_teleop deadman_teleop` — 누르고 있는 동안만 주행, space/Esc 정지(명령 즉시 0, diff_cont가 0.3 m/s²로 감속 — D-16). 급정지는 E-stop. 실험 전에는 종료(실행기가 다른 cmd_vel 발행자를 거부).
5. 실험: `ros2 run oroha_experiment runner`(별도 터미널) → `oroha_exp run --path square --side 1.0 --v 0.2 --rig-state on_ground` → 조건 확인 → 안내된 시작 위치에 로봇 → Enter 시작, space/Esc/Ctrl-C 중단, n 메모 → 종점 줄자 측정 → `oroha_exp gt --run <RUN_ID> --x … --y …`.
6. 추출·기록: `oroha_export_csv data/runs/<RUN_ID>` → `oroha_ledger --check` → 시험 기록(`records/tests/`).
7. 논문 묶음: `oroha_paper_export --runs usable` → `paper_export/<ts>/CHECK.md` 확인.

## 통신 두절·재연결 (개요 §4, T20260929-04에서 확인)

**간헐 두절(MD400 수신 잠김, T20260929-07)**: 한 MD400이 가끔 요청을 무시하는 상태에 빠진다. 플러그인 패치 0002가 실패한 장치를 즉시 재시도해 같은 주기에 복구한다(로그 `immediate retry ok`, 주기 1회 ≈0.1 s 지연). 재시도도 실패가 5회 이어지면 아래 절차.

**제자리 복구(M7, `oroha_hw_recover`)**: 재시도도 5회 연속 실패해 하드웨어가 ERROR로 내려간 경우, 런치를 다시 띄우지 않고 복구한다. ① E-stop 누른 채 유지 → ② 원인 해결(케이블·전원) → ③ `oroha_hw_recover`(컴포넌트 configure·activate, 컨트롤러 재활성화, `/joint_states` 복귀 확인; 종료코드 0 = 복구) → ④ 바퀴 0 확인 후 E-stop 해제. 실물 고장 주입(RS485에 0xFF 2 s)으로 확인: 2.3 s에 ERROR(컴포넌트 `unconfigured`, 컨트롤러 `inactive`) → 복구 0.4 s → 정상 주행(T20261006-04). 실패하면 아래처럼 런치를 다시 띄운다.

RS485가 끊기면 플러그인은 약 0.4 s(5회 실패) 뒤 하드웨어를 ERROR → unconfigured로 내리고 컨트롤러를 끄지만, **MD400은 마지막 명령으로 계속 돈다 — E-stop만이 멈춘다.** 복구: E-stop 누른 채 유지 → 케이블 재연결(링크 `/dev/oroha_md400`·latency 1 ms는 udev가 자동 적용) → 런치 Ctrl-C(안전망이 양쪽 정지) 또는 `oroha_md_stop` → E-stop 해제 → 런치 재기동.
