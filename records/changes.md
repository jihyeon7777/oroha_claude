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
