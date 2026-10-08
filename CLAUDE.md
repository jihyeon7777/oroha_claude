---
type: software-development-instructions
status: canonical
last_updated: 2026-09-22
tags: [OROHA, Claude-Code, ROS2, handoff]
---

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# OROHA 개발 시작 안내

OROHA는 HardwareX 논문으로 공개할 4륜 skid-steer 연구 플랫폼이다. 이 문서는 **2026-09-11 사용자 확정사항을 담은 Pi 5용 인계본**이다. 사용자와의 대화는 한국어로 진행한다.

## 시작하기

이 파일과 [개발_개요.md](개발_개요.md), [하드웨어_확인표.md](하드웨어_확인표.md), [계측_GT_연동.md](계측_GT_연동.md)를 Pi 5의 `oroha_ros2` 개발 디렉터리 루트에 함께 두고 여기서 Claude Code를 시작한다. 개발 개요를 먼저 읽고, 장치를 연결·설정할 때 하드웨어 확인표를, 계측·기록·내보내기를 작업할 때 연동 문서를 읽는다. 네 문서에 개발 시작에 필요한 기준을 요약했다.

**Claude Code 실행, 코드 작성, 빌드, 실물 시험은 모터와 필요한 센서가 연결된 Raspberry Pi 5에서 직접 진행한다.** 목표 환경은 Ubuntu 24.04＋ROS2 Jazzy이며, 실제 설치·연결 상태부터 확인한다. 계측 펌웨어는 별도 Raspberry Pi Pico(RP2040)에서 동작한다.

## 작업 방식

- 환경·연결 장치·기존 작업공간·설정을 조사하고 시작 상태를 짧게 정리한다. 모터 ID·좌우·부호와 센서 채널을 하드웨어 확인표의 초기값과 실물에 대조하고, 확인·변경한 값은 확인표에 갱신한다.
- 개발 개요의 모터·UM7 GitHub 저장소를 기준으로 구현한다. 기존 clone이 있으면 위치·버전·변경사항을 확인하고, 필요하면 소스를 준비한다. 사용할 커밋과 실제 빌드 방법을 기록한다.
- **작은 기능 구현 → 연결된 장치에서 시험 → 결과 확인·수정 → 통합 시험**을 반복한다. 모터·키보드·계측·실험 기록을 단계적으로 연결하고, 변경의 영향을 받는 기능을 다시 확인한다.
- 코드 구현 상태와 실물 확인 상태를 각각 남긴다. 문제를 수정하면 이전 시험과 재시험을 연결한다.
- 확인 가능한 사항은 먼저 조사한다. 사용자 의도, 실제 조작·시험 조건, 문서와 장치의 불일치처럼 판단이 필요한 사항은 해당 단계에서 사용자에게 질문하고 결정 내용을 갱신한다.

## 구현 재량과 기록

확정 범위와 데이터의 의미를 유지하면서 패키지 구성·토픽·프로토콜·언어·알고리즘은 개발 중 구체화한다. 사용자는 간결한 목표와 확인사항 중심의 문서를 원한다. 세부 설계와 동작 조건은 실제 시험과 대화를 통해 정리한다.

개발 저장소에 변경 목적, 설계 결정, 문제·수정, 시험 ID·조건·결과·로그 위치를 짧게 누적한다. 실제 코드·펌웨어·드라이버·설정·교정 버전을 시험과 연결하고, 미커밋 변경도 재구성할 수 있게 보관한다. 시험은 통과·실패·중단·미검증 등의 상태와 근거를 함께 남긴다.

**논문용 기록 추출도 개발 산출물이다.** [계측·GT 연동 문서](계측_GT_연동.md)에 따라 개발 중 내보내기 명령 또는 도구를 마련한다. 주요 기능 검증 후 논문용 요약을 갱신하고, 요청 시 `paper_export/<날짜_시각>/`에 요약·시험 목록·재현 자료·근거 로그를 함께 모은다. 사용자는 이를 논문 작성에 사용한다.

이 인계 시점의 산출물은 문서 네 파일이다. 로봇 소프트웨어와 내보내기 도구의 구현·실물 검증은 Pi 5에서 진행할 작업이다. 이후 구현 사실과 시험 결과는 개발 저장소에서 갱신한다.

## 워크스페이스 상태 (2026-09-28)

이 폴더는 **colcon 워크스페이스 겸 개발 기록 저장소**(git, `main`)다. 현재 계획은 `/home/oroha/.claude/plans/agile-enchanting-shell.md`(09-28 재검토판), 세션 시작점은 [records/next_session.md](records/next_session.md), 변경·결정·시험 기록은 [records/](records/)에 있다.

```
oroha.repos            외부 소스 고정: TaesuYim/mdrobot_motor_driver v1.4.0 (c5c1f3f), jihyeon7777/um7_driver (9a34258)
src/external/          위 checkout — 커밋하지 않음, 직접 수정 금지(patches/ 로만: mdrobot 0001 status·read_seq, 0002 즉시 재시도, 0003 DI)
src/oroha_msgs         PowerSample·ExperimentEvent/Status·ArmExperiment/AddNote
src/oroha_description  4륜(조인트 motor_L/motor_R 2개) xacro, use_mock_hardware 스위치, test/test_xacro.py
src/oroha_bringup      robot.launch.py(종료·크래시 시 MD400 정지 안전망), config/oroha_controllers.yaml(실측값), test/test_config.py
src/oroha_power        Pico 계측 노드(2.0.0), port_guard, config/calibration/sensing-20260828.yaml(교정 단일 출처)
src/oroha_experiment   profiles.py(시간 기반 경로·spot·방 크기 검사), guards.py(바퀴별 가드), runner_node(params.yaml·이벤트 미러), cli(oroha_exp)
src/oroha_tools        preflight·md_stop·direction_check·wheel_push·bus_probe·hw_recover·versions·export_csv(+power_analysis)·ledger·paper_export·verify_export
src/oroha_teleop       deadman_teleop (사용자 터미널 전용)
firmware/pico/         MicroPython main.py (oroha-bench-1.2, sha256 bace9505…; 1.1은 archive/)
records/               changes.md · decisions.md · tests/tests.csv + T*.md(+<ID>/ 증거) · calibration/ · preflight/
data/runs/<RUN_ID>/    실험 실행(meta·events·versions 커밋, bag·csv 미커밋); data/tests/<ID>/bag 시험 bag(미커밋)
setup/                 install_system.sh(sudo) · bootstrap.sh · env.sh · udev(FTDI latency 1 ms 포함) · chrony
```

- 장치(udev): `/dev/oroha_md400`(FTDI RS485, MD400 id1=우 id2=좌, 둘 다 fw v8.6), `/dev/oroha_pico`(`oroha-bench-1.2`, 검증됨), `/dev/oroha_um7`(CP2102, UM7 연결·**섀시 미고정**).
- 설치: ros2_control 4.48.0·ros2_controllers 4.42.1·xacro·chrony·Asia/Seoul. 시험 공간은 **3 m×3 m 방**(접지 시험·경로 크기는 방 크기 검사 통과 값만).
- 이전 팀 작업물은 `/home/oroha/oroha/`에 그대로 있다: `mdrobot_motor_driver/`(jihyeon7777 bringup 브랜치 `0aec730` — 실측 yaml·`oroha_fw/`·`test/` 실물 스크립트·`docs/hardware_test_*.md`)와 `oroha_handoff_20260910/`(확정 상수·보고서 7편·원시 로그·MANIFEST). 값·근거를 옮길 때만 참조한다.

## 자주 쓰는 명령

```bash
source setup/env.sh                      # 모든 터미널·프로세스에서 먼저 (ROS + install + ROS_DOMAIN_ID=42 + 도구 PATH)
bash setup/bootstrap.sh                  # vcs import(고정 커밋) + patches + rosdep
colcon build --symlink-install --packages-up-to oroha_bringup oroha_experiment oroha_tools oroha_teleop
colcon test --packages-select oroha_description oroha_bringup oroha_experiment oroha_power oroha_tools && colcon test-result --verbose

# ROS 없는 단위테스트 (빠름; oroha_tools는 PYTHONPATH=.:../oroha_power)
(cd src/oroha_experiment && python3 -m pytest -q)         # 경로 기하·spot·방 크기
(cd src/oroha_power && python3 -m pytest -q)              # Pico 프로토콜·환산·포트 점유
python3 -m pytest -q src/oroha_description/test src/oroha_bringup/test   # URDF 렌더·설정 규칙 (source /opt/ros/jazzy 필요)
(cd src/external/mdrobot_motor_driver && python3 -m pytest -q)

oroha_preflight [--sec 15]               # 런치 전(포트 점유 시 거부) → records/preflight/*.json
ros2 launch oroha_bringup robot.launch.py [use_mock_hardware:=true] [power:=false] [imu:=true] [rviz:=true]
oroha_md_stop                            # 비상: 양쪽 MD400 VEL_CMD 0·stop·torque_off (E-stop 해제 전에)
ros2 topic pub -r 10 -t 30 /diff_cont/cmd_vel geometry_msgs/msg/TwistStamped "{header: auto, twist: {linear: {x: 0.1}}}"   # 3 s 뒤 스스로 끝남
ros2 run oroha_tools oroha_direction_check --id 1 --yes [--sec 10 --resend --test-id T…]   # 바퀴 띄우고, 모터 돈다
oroha_wheel_push --phase start   →(사용자 밀기)→   oroha_wheel_push --phase end --revs 3
bash setup/drive.sh [--record] [--imu]                                # 키보드 주행 한 번에: 런치(뒤) + 텔레옵(앞), 종료 시 안전 정지 (사용자 터미널)
ros2 run oroha_teleop deadman_teleop                                  # 텔레옵만 (런치가 이미 떠 있을 때, TTY 필요)
ros2 run oroha_experiment runner [--ros-args -p require_*:=false]     # 실험 실행기
oroha_exp run --path square --side 1.0 --v 0.2 --rig-state lifted [--yes]   # 접지는 사용자 터미널에서 1회씩
oroha_exp conditions --set surface=... ; oroha_exp gt --run <RUN_ID> --x 1.47 --y -0.03
oroha_export_csv data/runs/<RUN_ID>; oroha_ledger --check             # CSV 추출, 실행·시험 목록 점검
oroha_paper_export --runs usable; oroha_verify_export paper_export/<ts>
oroha_profile s_curve --radius 0.4 --v 0.2                            # 프로파일·필요 공간 확인
oroha_versions --out records/tests/<ID>/                              # 시험별 버전·미커밋 diff 스냅샷
oroha_hw_recover [--dry-run]                                          # 통신 ERROR 뒤 제자리 복구(E-stop 누른 채)
oroha_exp note --manual-move "…"                                      # 실행 중 손으로 건드림 → MANUAL_MOVE (대화형 실행 중 m 키)
oroha_bus_probe --mode steps --resend --period 0.1 --sec 300 --on-lock diag --yes --out <csv>   # 런치 없이 RS485 트랜잭션 전수 기록(바퀴 돈다)
```

## 실물 시험 규칙

- MD400에는 통신 워치독이 없다. 호스트가 명령을 못 보내면(런치 Ctrl-C·크래시·RS485 분리) 마지막 명령이 남는다. 런치 안전망과 `oroha_md_stop`이 있어도 **E-stop은 항상 사용자 손에** 둔다. Claude의 도구 호출은 실시간 정지 경로가 아니다.
- 순서: 바퀴 띄움 → E-stop 손에 → **스스로 끝나는** 명령만 실행(`ros2 topic pub -t N`, 시간 제한 프로파일, `direction_check --sec`) → 사용자 관측 보고 → 기록. 바퀴를 띄운 회전 시험은 사용자 승인 없이 진행(2026-09-29 사용자 지시); 접지 주행과 사용자 손이 필요한 시험(E-stop 조작·관측)은 조율한다. 무기한 발행(`-r` 단독)·타임아웃 없는 컨트롤러 금지.
- E-stop: 누르면 0.13~0.14 s 안에 정지(코스팅). **해제하면 다음 명령이 오는 즉시 다시 돈다**(ros2_control은 매 주기 명령) → 해제 전에 명령 소스(텔레옵·실행기)를 멈추고 0 확인(`oroha_md_stop` 또는 런치 종료). T20260929-01.
- 속도 명령 전에 `enable()`이 필요하다(플러그인은 `auto_enable`). **`use_limit_sw`는 1**: E-stop은 CTRL 정지 게이트(6·8번, 2극 NC)로만 동작하고 MD400은 0이면 이를 무시한다(T20260928-06 실패). 현재 결선에서는 1이어도 양방향 주행 가능(T20260929-01). 옛 "1이면 역방향 차단"은 8번만 결선됐던 07-29 기준.
- preflight·direction_check·wheel_push는 포트를 연 다른 프로세스가 있으면 거부한다 — 런치 전에 쓴다. 모든 프로세스는 `source setup/env.sh`(ROS_DOMAIN_ID=42)로 띄운다.
- Claude가 띄우는 장시간 프로세스(런치·노드)는 **`pid=$(setup/bg.sh <log> <명령…>)`**로 띄우고 **`kill -INT -- -$pid`**(프로세스 그룹 전체)로 끈다(사용자 Ctrl-C와 같음). `ros2 run`은 래퍼 뒤에 실제 노드가 자식으로 있어 pid 하나에만 보내면 노드가 남는다(T20260929-06). 비대화형 셸의 백그라운드 작업은 SIGINT가 무시로 상속돼 `ros2 launch`가 `kill -INT`를 무시하고, **SIGTERM을 받으면 자식을 정리하지 않고 죽는다**(런치 안전망도 못 돎) — 런치에 SIGTERM 금지. 찾기는 `pgrep -f '[p]attern'`처럼 셸 자신과 일치하지 않는 패턴으로만(`pkill -f` 금지). 시험 전 `ros2 node list`로 중복 노드가 없는지 본다.
- 시험 하나가 끝나면 `records/tests/tests.csv`에 행을 더하고 `records/tests/T<KST 날짜>-<NN>.md`(+ `records/tests/<ID>/`에 `oroha_versions --out`·로그)를 남긴다. 상태 어휘: 통과/실패/중단/미검증, 종결/열림/다시 열림/재현 실패/미수렴. ID·좌우·부호는 [하드웨어_확인표.md](하드웨어_확인표.md) §2·§5에 시험 ID와 함께 적는다.
