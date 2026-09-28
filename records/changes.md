# 변경·결정 기록

날짜별로 변경 목적, 설계 결정, 문제·수정, 관련 시험 ID를 짧게 누적한다. 시험 상세는 `tests/<TEST_ID>.md`, 시험 목록은 `tests/tests.csv`.

## 2026-09-22

- 워크스페이스 시작. 이 폴더(`/home/oroha/oroha_claude`)를 colcon 워크스페이스 겸 개발 저장소로 만들고 첫 커밋(`4bd0f64`).
- 기준 저장소 결정: 모터 = TaesuYim/mdrobot_motor_driver **v1.4.0 (`c5c1f3f`)**, IMU = jihyeon7777/um7_driver **`9a34258`**. `oroha.repos`로 고정, `vcs export --exact`로 확인.
  - 근거: 개발_개요.md가 공개판을 지목했고, Pi의 기존 clone(jihyeon7777 bringup 브랜치 `0aec730`, ros2_control 0.2.4)은 코드 라인이 갈라져 있었다. 실측 설정값·펌웨어·시험 근거만 bringup 브랜치에서 옮긴다.
- Pico 펌웨어는 MicroPython `oroha-bench-1.1` 유지. `firmware/pico/main.py` sha256 `9ce752a9…` = 인계 문서의 플래시본과 일치.
- 실험 경로 명령은 시간 기반 개방루프 속도 프로파일(램프 포함)로 결정. `oroha_experiment/profiles.py` + 기하 단위테스트 11건 통과.
- Pi 시간대 Asia/Seoul로 변경하기로 결정(`setup/install_system.sh`, 사용자 실행 필요). 기록은 UTC + 시간대.
- `oroha_power` 2.0.0: 기존 1.3.1 노드를 `oroha_msgs/PowerSample`(raw+환산+calib_id)로 포팅. 시간 기준을 노드 시계 하나로 통일, 스테일 감지, 영점 서비스 비블로킹, launch bool 파라미터 타입 버그 수정, `simulate` 모드 추가.
- 미해결(사용자 실행 대기): `sudo bash setup/install_system.sh` — ros2_control·xacro·chrony·udev·시간대.
- 빌드·시험: T20260922-01(빌드·단위테스트 통과), T20260922-02(`oroha_power` simulate 통과), T20260922-03(**실물 preflight 22/22 통과** — MD400 v8.6 ×2, Pico oroha-bench-1.1 50 Hz). 커밋 `7fab197`.
- `setup/env.sh`가 ament_python 콘솔 스크립트(`oroha_preflight`, `oroha_exp` 등)를 PATH에 올리도록 함.
- **결함·수정**: 실물 Pico에서 `oroha_power`가 `t_us` 음수로 사망(T20260922-04). 펌웨어 `mono_us()` 유휴 누적 누락 → **oroha-bench-1.2**로 플래시(sha `bace9505…`, 1.1은 archive 보관). `PowerSample.t_us` int64·min/max int16, 노드 프레임 방어.
- 실험 실행기(`oroha_experiment/runner_node.py`)·CLI(`oroha_exp`)·bag→CSV 추출(`oroha_export_csv`) 1차 구현. mock 파이프라인 T20260922-06 통과. 실물 Pico 단독 T20260922-05 통과(열림: overrun 4 %, 영점 직후 편차).
- 교훈: 시험 명령에서 `source … && … &`는 체인 전체를 백그라운드 서브셸로 보내고, `pkill -f`/`pgrep -f`는 셸 자신의 명령줄과 일치할 수 있다 → 노드는 `(setsid … &)`로 띄우고 `pgrep -f '^/usr/bin/python3 .*[p]attern'`으로만 정리한다. 중복 노드는 서비스 응답을 뒤섞는다(`ros2 node list`로 확인).
- T-05b: 영점 후 정지 전류 −0.2 LSB, overrun 0.6 %(3.9 s 주기) → T20260922-05 종결. `oroha_ledger`·`oroha_paper_export`·`oroha_verify_export` 작성, docs/(frames·time_sync·operation) 초안.
- T20260922-07: 논문 묶음 생성·검증 통과(음성 시험 포함). `oroha_teleop/deadman_teleop`(load_manual 데드맨 포팅) 작성·빌드, 실물 미검증.
- **대기 중**: `sudo bash setup/install_system.sh`(ros2_control·xacro·chrony·udev·Asia/Seoul) — 0단계 빌드 관문·1단계 모터 시험이 여기에 막혀 있음.
- T-04b: 펌웨어 1.2, 560 s 유휴 후 `t_us` 연속(간격 560.3 s, 단조) → T20260922-04 종결.

## 2026-09-23 (KST; 시간대 변경 후)

- 사용자가 `setup/install_system.sh` 실행: ros2_control 4.48.0·ros2_controllers 4.42.1·xacro·chrony·udev(`/dev/oroha_*` 3개)·Asia/Seoul 확인. UM7 USB 연결(CP2102→`/dev/oroha_um7`), 섀시 미고정. 모터 시험 준비(바퀴 띄움·E-stop)됨.
- 배터리 충전을 위해 Pi 종료. 빌드·시험은 하지 않고 [next_session.md](next_session.md)에 시작점·순서를 정리. 미빌드: `mdrobot_ros2_control`·`oroha_description`·`oroha_bringup`.

## 2026-09-28 (KST)

- **계획 재검토**(설치 패키지·v1.4.0 소스·이전 보고서 대조, xacro 메모리 렌더, 교차 검토): 첫 주행 전 필수 5건(C1 xacro `reverse`가 `True`로 렌더링돼 왼쪽 reverse 미적용, C2 E-stop 해제 거동 미확인, C3 명령 잔류—on_shutdown 없음·RS485 분리·SIGTERM, C4 FTDI latency 16 ms, C5 3 m 방 접지 경로 안전)과 H1~H10을 찾아 세션 A/B/C로 재편. 사용자 결정 D-08(3 m×3 m), D-09(patches/).
- A0 코드 수정: xacro `$(arg)` + URDF·설정 테스트(수정 전 xacro에서 실패 확인), `oroha_md_stop`·런치 안전망, direction_check(SIGTERM 정지, USE_LIMIT_SW 0, ENC_PPR, 타임라인·정지/재출발 판정, --resend), wheel_push `--phase`, preflight(시간 동기·latency·포트·레지스터), port_guard, 컨트롤러 설정(max_deceleration·publish_rate 10·publish_limited_velocity·base_footprint·wheel_vel_cont 제거), runner/power/teleop/cli 신호 처리·RELIABLE QoS, 실행기 한계 0.35/1.2·방 크기 검사·spot·`--rig-state`·Ctrl-C 중단·`gt`, 원장 usable 규칙(mock 실행 제외), 오도메트리 비교(시작 자세 기준·yaw 연속). setup: FTDI latency udev·rviz_imu_plugin.
- T20260928-01 빌드·테스트 관문 통과(11 패키지 56.5 s 경고 0, 테스트 316 실패 0 — 패치 없는 v1.4.0이 ros2_control 4.48에서 그대로 동작).
- T20260928-02 mock bringup 통과(10 Hz, 단일 TF 루트, diff_cont 구독 QoS BEST_EFFORT 확인, 명령 0.68 m 예측 일치). 런치를 Claude 셸에서 띄우면 SIGINT 무시·SIGTERM 시 자식 미정리 → `setup/bg.sh` 추가, 규칙 문서화.
- T20260928-03 실행기 mock: bag 녹화기 늦은 구독(3.15 s)·transient_local 캐시 이벤트로 추출 t=0이 틀리던 결함 수정(녹화기 구독 대기, run_id의 START header stamp 사용). SIGINT·Ctrl-C 정리 경로, CLI "RUNNER LOST", cmd_vel_out 추출 추가. env.sh에서 사용 중단된 ROS_LOCALHOST_ONLY 제거.
- T20260928-05 방향 점검 통과: id1 오른쪽 전진·id2 왼쪽 후진(사용자 관측, 500 rpm). 100 rpm은 바퀴 2.9 rpm이라 육안 판별 불가 → direction_check rpm 상한 600.
- **T20260928-06 실패(치명)**: USE_LIMIT_SW 0에서 E-stop을 눌러도 모터가 계속 돎(500 rpm, 10 s). 모든 모터 작업 중단. E-stop 결선·방식 결정 필요.
