# OROHA — ROS 2 workspace

4륜 skid-steer 연구 플랫폼 OROHA의 ROS 2 Jazzy 워크스페이스이자 개발 기록 저장소다. 목표·범위는 [개발_개요.md](개발_개요.md), 실측값은 [하드웨어_확인표.md](하드웨어_확인표.md), 계측·기록·내보내기 기준은 [계측_GT_연동.md](계측_GT_연동.md), 작업 방식은 [CLAUDE.md](CLAUDE.md)에 있다.

## 빠른 시작 (Raspberry Pi 5, Ubuntu 24.04 + ROS 2 Jazzy)

```bash
sudo bash setup/install_system.sh      # 1회: ros2_control·xacro·chrony, udev(/dev/oroha_*), 시간대 Asia/Seoul
bash setup/bootstrap.sh                # 외부 소스(oroha.repos 고정 커밋) import + 패치 + rosdep
source setup/env.sh
colcon build --symlink-install --packages-up-to oroha_bringup oroha_experiment oroha_tools
source setup/env.sh

oroha_preflight                        # 런치 전 장치 점검 (RS485 포트를 독점하므로 런치 전에)
ros2 launch oroha_bringup robot.launch.py            # 실물. use_mock_hardware:=true 면 장치 없이
ros2 run oroha_teleop deadman_teleop                 # 키보드 주행(데드맨, 별도 터미널)
ros2 run oroha_experiment runner                     # 실험 실행기(별도 터미널)
oroha_exp run --path straight --length 2.0 --v 0.2  # 논문 실험 실행·기록 (조건 입력 → Enter 시작)
oroha_export_csv data/runs/<RUN_ID> && oroha_ledger  # bag → CSV, 실행 목록 갱신
oroha_paper_export --runs usable                     # paper_export/<ts>/ + CHECK.md (검증 포함)
```

## 구성

| 경로 | 내용 |
|---|---|
| `oroha.repos` | 외부 소스 고정: `TaesuYim/mdrobot_motor_driver` v1.4.0, `jihyeon7777/um7_driver` |
| `src/external/` | 위 소스의 checkout (커밋하지 않음). 수정은 `patches/`로만 |
| `src/oroha_*` | OROHA 패키지: msgs · description · bringup · power · teleop · experiment · tools |
| `firmware/pico/` | Pico 계측 펌웨어(MicroPython, `oroha-bench-1.2`)와 sha256, 1.1은 `archive/` |
| `setup/` | 시스템 설치·환경 스크립트, udev·chrony 설정 |
| `records/` | 개발 기록: 변경·결정·시험 목록·교정값·preflight 결과·논문용 요약 |
| `data/runs/<RUN_ID>/` | 실험 실행별 메타·이벤트·버전(커밋) + bag·csv(커밋 안 함) |
| `docs/` | 좌표계·시간 동기·운용·CSV 열 설명 |
| `paper_export/` | 요청 시 생성하는 논문 근거자료 묶음 (커밋 안 함) |

## 안전

모터를 움직이는 시험은 **바퀴를 띄운 상태**에서 먼저 하고, E-stop이 손에 닿는 곳에 있어야 한다. MD400에는 통신 워치독이 없어 호스트가 명령을 못 보내면 마지막 명령이 남는다 — 런치는 `ros2_control_node`가 끝나면 양쪽 MD400을 멈추고, 수동 정지는 `oroha_md_stop`. 모든 모션 명령은 스스로 끝나야 한다. `use_limit_sw`는 0을 유지한다(1이면 왼쪽 바퀴의 음수 명령이 무알람 차단된다). 자세한 규칙은 [CLAUDE.md](CLAUDE.md)와 [docs/operation.md](docs/operation.md).
