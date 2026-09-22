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

## 워크스페이스 상태 (2026-09-22)

이 폴더는 **colcon 워크스페이스 겸 개발 기록 저장소**(git, `main`)다. 승인된 단계별 계획은 `/home/oroha/.claude/plans/agile-enchanting-shell.md`, 변경·결정·시험 기록은 [records/](records/)에 있다.

```
oroha.repos            외부 소스 고정: TaesuYim/mdrobot_motor_driver v1.4.0 (c5c1f3f), jihyeon7777/um7_driver (9a34258)
src/external/          위 checkout — 커밋하지 않음, 직접 수정 금지(patches/ 로만)
src/oroha_msgs         PowerSample·ExperimentEvent/Status·ArmExperiment/AddNote
src/oroha_description  4륜(조인트 motor_L/motor_R 2개) xacro, use_mock_hardware 스위치
src/oroha_bringup      robot.launch.py, config/oroha_controllers.yaml(실측값), power.yaml, um7.yaml
src/oroha_power        Pico 계측 노드(2.0.0), config/calibration/sensing-20260828.yaml(교정 단일 출처)
src/oroha_experiment   profiles.py(시간 기반 경로), runner/cli(3단계)
src/oroha_tools        oroha_preflight·oroha_versions·oroha_direction_check·oroha_wheel_push·export/ledger/paper_export(4단계)
firmware/pico/         MicroPython main.py (oroha-bench-1.1, sha256 9ce752a9…)
records/               changes.md · decisions.md · tests/tests.csv + T*.md · calibration/ · preflight/
data/runs/<RUN_ID>/    실험 실행(meta·events·versions 커밋, bag·csv 미커밋)
setup/                 install_system.sh(sudo) · bootstrap.sh · env.sh · udev · chrony
```

- 장치: `/dev/oroha_md400`(FTDI RS485, MD400 id1=우 id2=좌, 둘 다 fw v8.6), `/dev/oroha_pico`(MicroPython, `oroha-bench-1.1` 상주). udev 적용 전에는 `/dev/serial/by-id/…` 경로를 `--md-port`/`--pico-port`/`port:=`로 넘긴다. **UM7 미연결.**
- 시스템 설치(`sudo bash setup/install_system.sh`: ros2_control·xacro·chrony·udev·Asia/Seoul)는 사용자가 터미널에서 실행한다. 설치 전에는 `mdrobot_ros2_control`·`oroha_description`·`oroha_bringup` 빌드가 불가.
- 이전 팀 작업물은 `/home/oroha/oroha/`에 그대로 있다: `mdrobot_motor_driver/`(jihyeon7777 bringup 브랜치 `0aec730` — 실측 yaml·`oroha_fw/`·`test/` 실물 스크립트·`docs/hardware_test_*.md`)와 `oroha_handoff_20260910/`(확정 상수·보고서 7편·원시 로그·MANIFEST). 값·근거를 옮길 때만 참조한다.

## 자주 쓰는 명령

```bash
source setup/env.sh                      # ROS + install + ROS_DOMAIN_ID=42 + 도구 PATH
bash setup/bootstrap.sh                  # vcs import(고정 커밋) + patches + rosdep
colcon build --symlink-install --packages-up-to oroha_bringup oroha_experiment oroha_tools
colcon build --symlink-install --packages-up-to oroha_power oroha_experiment oroha_tools   # ros2_control 없이 가능한 부분
colcon test --packages-select oroha_experiment oroha_power oroha_tools && colcon test-result --verbose

# ROS 없는 단위테스트 (빠름)
(cd src/oroha_experiment && python3 -m pytest -q)         # 경로 프로파일 기하
(cd src/oroha_power && python3 -m pytest -q)              # Pico 프로토콜·환산
(cd src/external/mdrobot_motor_driver && python3 -m pytest -q)   # v1.4.0은 루트 pytest.ini 보유

oroha_preflight [--sec 15]               # 런치 전 읽기 전용 점검 → records/preflight/*.json (RS485 포트 독점)
ros2 launch oroha_bringup robot.launch.py [use_mock_hardware:=true] [power:=false] [imu:=true] [rviz:=true]
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel -p stamped:=true -p frame_id:=base_link -p speed:=0.2 -p turn:=0.5
ros2 run oroha_power power_node --ros-args -p simulate:=true          # 장치 없이
ros2 run oroha_teleop deadman_teleop                                  # 데드맨 키보드 주행(실물 미검증)
ros2 run oroha_experiment runner [--ros-args -p require_*:=false]     # 실험 실행기; mock 시험은 검사 비활성
oroha_exp run --path square --side 1.0 --v 0.2 [--repeats 3] [--yes]  # 실험 실행 → data/runs/<RUN_ID>/
oroha_export_csv data/runs/<RUN_ID>; oroha_ledger --check             # CSV 추출, 실행·시험 목록 점검
oroha_paper_export --runs usable; oroha_verify_export paper_export/<ts>
ros2 run oroha_tools oroha_direction_check --id 1                     # 바퀴 띄우고, 모터 돈다
oroha_profile square --side 1.0 --v 0.2 [--csv out.csv]              # 프로파일 확인
oroha_versions [--out <dir>]             # 버전·미커밋 diff 스냅샷
```

## 실물 시험 전 확인

- MD400에는 통신 워치독이 없다. 정지는 `diff_cont`의 `cmd_vel_timeout 0.5`와 물리 E-stop뿐이므로, 모터 시험은 **바퀴를 띄운 상태**에서 먼저 하고 E-stop을 손에 둔다.
- 속도 명령 전에 `enable()`(`PID_UI_COM(78)=1` + `PID_START_STOP(100)=1`)이 없으면 명령은 echo되지만 모터는 0에 머문다(플러그인은 `auto_enable`로 처리).
- `use_limit_sw`는 0 고정. 1이면 음수 명령이 차단돼 왼쪽(reverse) 바퀴가 무알람으로 서 버린다.
- ID·좌우·부호는 `oroha_direction_check`로 실물 대조하고 [하드웨어_확인표.md](하드웨어_확인표.md) §2·§5에 시험 ID와 함께 적는다. 계측 상수와 기각된 옛 값은 §3·§6과 `records/calibration/`이 단일 출처다.
- 시험 명령에서 노드는 `(setsid ros2 run … > log 2>&1 &)`로 띄우고, 정리는 `for p in $(pgrep -f '^/usr/bin/python3 .*[p]attern'); do kill $p; done`처럼 셸 자신의 명령줄과 일치하지 않는 패턴으로만 한다(`pkill -f`는 셸을 죽이고, `source … && … &`는 체인 전체를 서브셸로 보낸다). 시험 전 `ros2 node list`로 중복 노드가 없는지 본다.
- 시험 하나가 끝나면 `records/tests/tests.csv`에 행을 더하고 `records/tests/T<날짜>-<NN>.md`에 목적·조건·절차·관측·판정·로그·버전을 남긴다. 상태 어휘: 통과/실패/중단/미검증, 종결/열림/다시 열림/재현 실패/미수렴.
