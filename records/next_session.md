# 다음 세션 시작점

갱신 2026-09-28 16:50 KST (세션 A 진행 중: A2·A3 완료, A4 중단). 계획 원문: `/home/oroha/.claude/plans/agile-enchanting-shell.md`. 이 파일은 세션이 끝날 때마다 덮어쓴다.

## 현재 상태

| 항목 | 상태 |
|---|---|
| 코드 | 재검토 A0 반영 + mock 시험 중 수정(녹화기 구독 대기, 추출 t=0 = 이 실행 START stamp, `setup/bg.sh`) |
| 빌드·mock | ✅ T20260928-01(빌드 11개·테스트 316 통과), T-02(mock bringup), T-03(실행기 mock·SIGINT·Ctrl-C) |
| preflight | T-04 중단: MD400 OK(`USE_EPOSI 0`, `ENC_PPR 1000` = 의도값), **Pico USB 무응답 → 사용자 재연결 후 재현 시험**(preflight 연속 2회, power 노드 기동·정지 반복) |
| FTDI latency | 스크립트 수정됨. **사용자 `sudo bash setup/install_system.sh` 재실행 필요**(확인: `cat /sys/class/tty/ttyUSB0/device/latency_timer` = 1) |
| 로봇 | 주행 이력 없음. 세션 A는 바퀴 띄운 상태만 |
| UM7 | 연결됨, 미고정 |

## 세션 A (접지 없음) — 시험 ID `T<KST 날짜>-NN`

| 단계 | 내용 | 누가 | 통과 기준 |
|---|---|---|---|
| A1 | sudo 재실행(latency 1 ms, rviz_imu_plugin) | 사용자 | latency 1 |
| A2 | 전체 빌드(패치 없는 v1.4.0 기준선, 백그라운드+로그 `records/tests/<ID>/`) + `colcon test` | Claude | 종료코드 0, `test_plugin_load`·xacro·config 테스트 통과 |
| A3 | mock bringup(`use_mock_hardware:=true power:=false`) + 실행기 mock(require_controller 켬: straight·spot·square, 실행 중 `kill -INT`) | Claude | 컨트롤러 active, odom ≈10 Hz, `view_frames` 단일 루트, `ros2 topic info -v /diff_cont/cmd_vel` 기록, odom 거리 이상값 ±5 %, SIGINT 실행 ABORTED + `metadata.yaml` + 고아 bag 0, 로그에 deprecation 없음 |
| A4 | `oroha_preflight` (Pico 재연결 후 재시험) | Claude | GO(latency·시간 동기·`USE_EPOSI 0`·포트·Pico 스트림) |
| A5 | 방향 점검 `--id 1`, `--id 2` (100 rpm, 3 s, `--yes`는 "진행" 후) | Claude 실행, 사용자 관측 | id1 오른쪽 전진, id2 왼쪽 후진 |
| A6 | **E1 E-stop 특성** `--sec 10`(latched), `--sec 10 --resend`(ros2_control처럼 재전송) — 도는 중 사용자 누름→해제 | 사용자 E-stop, Claude 기록 | 0 rpm ≤0.5 s, 해제 시 재출발 여부 기록 → `docs/operation.md` 해제 규칙. **못 세우면 모터 작업 중단** |
| A7 | 실물 런치(띄움, bag `data/tests/<ID>/bag`): 전진·CCW·바퀴별 조합(`-t` 제한), `/joint_states` 부호, Pico 전류 부호, CM 진단 주기 | Claude, 사용자 관측 | 부호·크기 정상, 실행시간 < 80 ms |
| A8 | 정지 행렬(런치는 `setup/bg.sh`로 띄워야 `kill -INT`가 Ctrl-C와 같음): 발행 중단 / runner `kill -9` / launch SIGINT(안전망) / `ros2_control_node` `kill -9`(안전망) / RS485 분리(계속 돎 → E-stop → 재연결 → `oroha_md_stop`) / 명령 중 E-stop | Claude + 사용자 | 호스트 개입 가능 시 ≤0.7 s 정지, 복구 절차 입증 |
| A9 | 띄운 실행기(계측 포함) straight 2 m·square 1 m, 실행 중 E-stop | Claude 준비, 사용자 E-stop | DONE, 정속 ±5 %, E-stop 후 ≤1.5 s FAIL, bag 확정 |
| A10 | 데드맨 텔레옵(사용자 터미널, 띄움); UM7 통신 점검(미고정) | 사용자 / Claude | 떼면 ≤0.3 s 감속 / checksum ≈0 |

세션 B(패치·전류 의미·접지 저속)와 C(UM7 장착·통합)는 계획 원문 참조.

## 결정 대기 (해당 세션에서)

UM7 장착 위치·방향 · git 원격과 bag 외부 백업 위치 · DMM Δ/VREF 측정 여부.
