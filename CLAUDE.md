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

## 시작 상태 (2026-09-22 실제 확인)

- 호스트는 Raspberry Pi 5 / Ubuntu 24.04.4 LTS / ROS 2 Jazzy(`/opt/ros/jazzy`, `ros-jazzy-desktop` 설치)다. `colcon`·`rosdep`·`vcs`·`cmake`·pyserial 3.5 사용 가능.
- **미설치: `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`, `ros-jazzy-xacro`.** ros2_control 런치 전에 설치가 필요하다. Pico 업로드용 `mpremote`·`picotool`도 없다.
- 연결 장치: `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG043HTG-if00-port0` → `ttyUSB0` (MD400 RS485), `usb-MicroPython_Board_in_FS_mode_e6616408435d4437-if00` → `ttyACM0` (Pico, MicroPython 상주 중). **UM7은 현재 열거되지 않는다** — 연결과 포트를 먼저 확인한다.
- 이 디렉터리(`/home/oroha/oroha_claude`)는 git 저장소가 아니며 문서 4편만 있다. 코드 작업공간은 아직 만들어지지 않았다.

## 기존 작업물 (이 디렉터리 밖, `/home/oroha/oroha/`)

| 위치 | 내용 |
|---|---|
| `mdrobot_motor_driver/` | colcon 워크스페이스 clone. 브랜치 `test/hardware-bringup-20260809`, 커밋 `0aec730`(2026-09-10). **자체 `CLAUDE.md`에 계층 구조·twin 모드·단위 정책·검증된 구동 시퀀스가 있으므로 모터 코드를 만지기 전에 그 파일을 읽는다.** |
| `mdrobot_motor_driver/oroha_fw/` | Pico MicroPython 펌웨어(`pico/main.py`, 50 Hz raw ADC CSV) + `oroha_power` ROS 2 패키지 + 벤치 도구. colcon `src/`에 포함되지 않는 별도 서브프로젝트 |
| `mdrobot_motor_driver/test/` | 유닛 테스트가 아니라 **실물 브링업 스크립트 모음** — 실행하면 모터가 실제로 돈다 |
| `oroha_handoff_20260910/` | 08-28 이후 확정 상수·세션 보고서 7편·원시 로그 201편·`MANIFEST.sha256`. `paper_export/` 묶음 구성의 선례로 쓸 수 있다 |

`개발_개요.md`는 모터 저장소를 `TaesuYim/mdrobot_motor_driver`로 적었으나 위 clone의 remote는 `jihyeon7777/mdrobot_motor_driver`다. 어느 쪽을 기준으로 할지 사용자에게 확인한다. UM7 드라이버(`jihyeon7777/um7_driver`)는 아직 clone되어 있지 않다.

## 자주 쓰는 명령 (기존 워크스페이스 기준)

```bash
source /opt/ros/jazzy/setup.bash
colcon build
colcon build --packages-select mdrobot_cpp mdrobot_ros2_control
colcon test --packages-select mdrobot mdrobot_cpp mdrobot_ros2_control && colcon test-result --verbose
```

하드웨어 없는 유닛 테스트 — 워크스페이스 루트에 `pytest.ini`가 없어 `PYTHONPATH` 없이 루트에서 실행하면 수집 단계에서 깨진다:

```bash
PYTHONPATH=src/mdrobot python3 -m pytest src/mdrobot/test -q
PYTHONPATH=src/mdrobot python3 -m pytest src/mdrobot/test/test_frame.py -q   # 단일 파일
PYTHONPATH=src/mdrobot python3 -m pytest src/mdrobot/test -k crc -q          # 이름으로 선별
colcon test --packages-select mdrobot_cpp --ctest-args -R test_frame         # 단일 gtest
```

실행·계측:

```bash
ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=twin   # OROHA는 twin (단채널 2대)
ros2 launch mdrobot_diffbot_example diffbot.launch.py                  # mock_components, 하드웨어 없이 확인

pip install mpremote
mpremote connect /dev/ttyACM0 fs cp oroha_fw/pico/main.py :main.py && mpremote connect /dev/ttyACM0 reset
mpremote connect /dev/ttyACM0 repl   # S 시작 / X 정지 / Z 영점 / C 설정 / G 통계, Ctrl-] 로 나감
```

포트·모터 ID·`reverse_*`·`counts_per_rev`·`update_rate`는 URDF가 아니라 `config/<device_type>_controllers.yaml`의 `mdrobot_hardware: ros__parameters` 섹션에서 바꾼다.

## 실물 시험 전 확인

- MD400에는 통신 워치독이 없다. 호스트가 멈추면 물리 E-stop만 남으므로 호스트 측 `cmd_vel_timeout`이 필수다.
- 속도 명령 전에 `enable()`(`PID_UI_COM(78)=1` + `PID_START_STOP(100)=1`)이 없으면 명령은 정상 echo되지만 모터는 0에 머문다.
- 첫 구동은 바퀴를 띄우고 한 대씩. ID·좌우·부호는 [하드웨어_확인표.md](하드웨어_확인표.md) §5의 방법으로 실물 대조한다.
- 계측 상수와 기각된 옛 값은 [하드웨어_확인표.md](하드웨어_확인표.md) §3·§6이 단일 출처다.
