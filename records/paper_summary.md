# 논문용 요약 (작성 중 — 각 수치에 시험 ID를 붙인다)

_HardwareX의 Software description / Operation instructions / Validation & characterization에 옮겨 쓸 수 있게 구성한다. 확인된 범위만 적고, 미검증은 미검증으로 둔다. 마지막 갱신 2026-09-22._

## 1. 소프트웨어 구성과 역할

| 구성 | 역할 | 근거 |
|---|---|---|
| ROS 2 Jazzy 워크스페이스 (`oroha_*` 7 패키지) | 기본 운용·계측·실험·기록 | 이 저장소 |
| `mdrobot_ros2_control` v1.4.0 (twin) | MD400 ×2 Modbus RTU 하드웨어 인터페이스, `diff_drive_controller`로 skid-steer 주행, 10 Hz | 외부, `oroha.repos` |
| `oroha_description`·`oroha_bringup` | 4륜 모델(조인트 2), 실측 설정값, 런치 | — (0단계 관문 T-… 예정) |
| `oroha_power` + Pico 펌웨어 `oroha-bench-1.2` | ACS37030 전류 ×2·버스전압 50 Hz, raw+환산+교정 ID 기록 | T20260922-03/04/05 |
| `um7_driver` | UM7 IMU (ENU) | 미연결 |
| `oroha_experiment` | 시간 기반 개방루프 경로(직선·원·사각·S), 실행기(이벤트·rosbag), CLI | T20260922-06 |
| `oroha_tools` | preflight, 버전 스냅샷, bag→CSV, 실행/시험 목록, 논문 묶음·검증 | T20260922-03/06/07 |

## 2. 운용 방법

[docs/operation.md](../docs/operation.md) 요약: preflight → 런치 → (키보드) → 실험 실행·기록 → CSV 추출 → 묶음 내보내기. 안전: 바퀴 띄우고 시작, E-stop, `cmd_vel_timeout 0.5 s`, MD400 통신 워치독 없음.

## 3. 주요 설계 결정

[records/decisions.md](decisions.md) D-01~D-07: 공개판 v1.4.0 기반·외부 소스 고정, MicroPython 펌웨어 유지, 시간 기반 프로파일, 커스텀 계측 메시지, 전용 description, udev 장치명, 단일 시간 기준.

## 4. 확인된 기능·성능·한계 (시험 ID)

| 항목 | 값·상태 | 조건 | 시험 |
|---|---|---|---|
| Pico 계측 발행률 | 50 Hz (표본간격 19.998 ms), seq 결번 0, overrun 0.6~0.8 % | 정지, Pi 5 USB CDC | T-03, T-05 |
| 정지 잡음 | σ 1.6 LSB ≈ 18 mA (목표 < 30 mA) | 정지, 15~20 s | T-03, T-05b |
| 영점 서비스 | `#ZERO` 적용, 이후 정지 전류 −0.2 LSB | 정지 | T-05b |
| 시간 정렬(Pico↔ROS) | min-filter 잔차 0.4 ms | — | T-05 |
| 버스전압 | 27.33 V (GP26), MD400 내장계 26.30/27.00 V(격차 +0.70 V, 계측 오프셋) | 정지 | T-03 |
| MD400 펌웨어 | id1·id2 DL=86 | — | T-03 |
| 실험 파이프라인 | 실행→bag(mcap)→CSV, 이벤트 8종 | mock | T-06 |
| 묶음 무결성 | MANIFEST 62/62, 결손 지목 | — | T-07 |
| 주행(twin diff-drive) | **미검증** (시스템 설치 대기) | — | 1단계 예정 |
| 오도메트리 정확도, 4륜 유효 트랙 | **미검증** | — | 1단계 예정 |
| UM7 | **미연결** | — | 2단계 예정 |

## 5. 한계·미해결

- 전류 절대값 교정 범위 0~1.2 A(무부하 3000 rpm까지), 게인 ±1.5 %, 런 간 ±2~3 %.
- 바닥 기울기·질량·적재·공기압·기온은 실행 조건 프롬프트로 기록(미측정 허용).
- 펌웨어 1.2: 560 s 유휴 후 `t_us` 연속성 확인(T-04b). 1.1 데이터는 스트림 중단 없는 실행에서 얻은 것이라 영향 없음.
