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

## 2026-09-29 (KST)

- **T20260929-01 통과**: 현재 E-stop 결선(2NC → CTRL 6+8번)에서 `USE_LIMIT_SW 1`이어도 두 컨트롤러 양방향 회전, E-stop 0.13~0.14 s 정지, 명령 재전송 시 해제 즉시 재출발. → 운용값 **`use_limit_sw: 1`**(yaml·xacro·테스트·direction_check·문서·확인표). T20260928-06은 이 시험으로 대체. 이전 팀의 "1이면 역방향 차단"은 8번만 결선된 07-29 조건.
- 사용자 지시: 바퀴를 띄운 회전 시험은 승인 없이 진행.
- **T20260929-02 통과**: ros2_control 첫 실물 주행(바퀴 띄움) — 명령 대비 속도 ±1 %, 부호 정확, reverse 적용 확인. 정상 종료는 4.48이 hardware deactivate로 정지시켜 `on_shutdown` 패치 불필요(재검토 C3⑤ 철회). 크래시(`kill -9`)는 안전망 없이 262 rpm 계속 → 안전망이 막음. 주기 88 ms로 FTDI 1 ms(A1) 필요.
- **T20260929-04 통과**: FTDI 1 ms 적용(주기 64 ms), ros2_control 운용 중 E-stop 정지·해제 시 재출발, RS485 분리 시 E-stop까지 계속 회전 — 복구 절차 문서화. 세션 A의 띄운 상태 안전 시험 완료.
- T20260929-05 중단: 사각 경로 회전 중 왼쪽 MD400 전원 순간 차단 추정(Pico 전류가 대기 이하, 카운터 리셋), 버스 전체 4 s 무응답, 오른쪽은 명령 유지. runner에 FAIL(하드웨어 링크 끊김) 시 Modbus 직접 정지 추가.
- T20260929-06: 두절은 간헐(사각 3회 중 2회, side3 감속 직후, 왼쪽 MD400 무전력 정황). 직접 정지 보완 동작 확인. 정지 규칙 kill -INT -- -$pid(ros2 run 래퍼 뒤 노드 잔존).
- **T20260929-07 통과(종결)**: 통신 두절 원인 = **MD400 수신 잠김**. 신규 `oroha_bus_probe`(런치 없이 두 MD400 직접 폴링, 트랜잭션 전수 기록, 잠김 진단 단계)로 ros2_control 없이 5회 재현. 잠긴 장치는 다른 장치 통신 직후의 요청을 모두 무시(무응답·명령 미실행, status 0, 카운터 유지), 휴지 뒤 요청 하나에 복구. 전원 문제 정황 없음(Pico). 프레임 간격 5 ms로도 발생. T-05·06의 "왼쪽 무전력" 해석 대체.
- 패치 `patches/external/mdrobot_motor_driver/0001`(joint별 `status`·`status2`·`read_seq` 상태 인터페이스 → `/dynamic_joint_states`, export `joint_diag.csv`), `0002`(실패 장치 즉시 재시도 `resync_retry`, `inter_frame_delay` 파라미터). xacro: 새 상태 인터페이스, `timeout 0.1`, `resync_retry true`. 재시도 후 띄운 사각 8/8 DONE(잠김 3회 복구). `.gitignore`: `data/tests/*/bag*/`.

## 2026-10-01 (KST)

- Pi 재부팅 후 preflight GO 28/28(장치 링크 유지, ttyUSB 번호는 바뀜 — 링크만 사용).
- **T20261001-01 통과**: 데드맨 텔레옵(띄움), 사용자 7항목 정상. bag: 해제→감속 0.106~0.153 s(≤0.3 s), 방향·크기 이론값 일치. 텔레옵에 해제 로그 추가. 종료 시 이중 SIGINT(그룹 Ctrl-C = 직접 + launch/`ros2 run` 전달)가 정리 단계를 끊는 문제 → `power_node`·`runner_node`·`deadman_teleop`이 정리 중 추가 신호 무시(재시험 통과). `um7_node`(외부)는 5 s 뒤 SIGTERM으로 끝남 — 안전 무관, 보류.
- **T20261001-02 통과**: UM7 통신(미고정) — 패킷 83 Hz, 체크섬 0, `/imu/data` 39.7 Hz, header stamp는 수신 시각 묶음. 세션 A 완료.
- **T20261001-03 통과(B2, H3)**: 전류 의미 분리. 노드 `i_*`는 항상 참 0 A 기준(+`rail_corr`, 명시적 `Z`에서만), boot zero 무시, 적용 내용은 `~/calibration_event`, 진단에 기준·baseline·범위 밖 수. `PowerSample`은 유지(기존 bag 호환). 신규 `power_analysis.py`: 실행의 정지 구간 → `rail_corr_run`·baseline → `di_*`·`i_*_abs`·분기별/합계 에너지(절대·증가분)·범위 밖 비율, 공통 모드(프로세스 시작 시 세 채널 동반 하강) 표본 제외. 띄운 계단 0.2~0.76 m/s 실측, MD400 effort는 Pico 대체 불가. 09-29 사각 8회 재추출: 증가분 에너지 변동 1.4 %.

## 2026-10-06 (KST)

- 모터 전원 OFF(preflight NO-GO: 버스 0.17 V, MD400 무응답) — 그 상태의 정지 raw(GP27 2030.65·GP28 2028.69)를 참 0 A 직접 측정값으로 보존.
- **T20261006-01 (mock 통과, 실물 열림)**: M2 바퀴별 가드(`guards.py`, FAIL 문구에 Pico 전류·MD400 status·원인 힌트), M3 ARM 때 `params.yaml`(노드 파라미터·URDF·하드웨어 컴포넌트, 서비스 호출), M4 이벤트(`PREFLIGHT_OK`·`ZERO`·`MANUAL_MOVE`·`/rosout` WARN 이상 미러), M7 `oroha_hw_recover`, CLI 서비스 대기 10 s·한 줄 오류, preflight USE_LIMIT_SW 문구 정정.
- **T20261006-02 통과**: 모터 전원 OFF(참 0 A) ↔ ON 정지 raw 차 +6.49/+6.60 LSB = 74/76 mA → 대기전류 가정 80 mA 유효(−5 mA).
- **T20261006-03 통과**: 실물 실행에서 `PREFLIGHT_OK`, `params.yaml` 노드 6개(UM7 포함)·URDF 하드웨어 설정, 가드 기하 diff_cont. T20261006-01 종결.
- **T20261006-04 통과**: RS485 0xFF 주입 2 s → 2.3 s에 ERROR(컴포넌트 unconfigured·컨트롤러 inactive) → `oroha_hw_recover` 0.4 s 복구 → 정상 주행. `docs/operation.md` 갱신.
- **T20261006-05 (B3)**: 사용자 보고 질량 약 50 kg, 외곽 0.80×0.53 m, 바닥 평평, 10 psi → 조건·확인표 반영. 방 크기 검사를 원(반대각) 모델에서 차체 사각형 모델로 변경(직진에서 0.16 m 낭비 제거): 직진 1.6 m(대각 2.0)·원 R0.75·사각 1.4·S R0.4가 들어감. `oroha_wheel_push --phase end`가 `--revs`와 `--dist`를 함께 받아 하중 아래 구름 둘레도 계산.
- **T20261006-06 통과(B4)**: 벽 방향 토크 오프 손밀기, 앞면 가운데 기준 1.500 m 왕복 → 0.7497 mm/count(−1.52 %) → `wheel-20261006` 확정, `wheel_radius 0.003580`(D-15). 타이어 접지 표시 방식(밀기 1·2)은 위치 오차로 제외. 오른쪽 둘레 +0.57 %(참고, 미적용).
- **T20261006-07 중단(B5 첫 접지 텔레옵)**: 오른쪽(141~145 s)·왼쪽(149~151 s) MD400이 알람 없이 전류 0으로 공회전(E-stop 신호와 같음) → 곡선 중 왼쪽 ALARM+OVER_LOAD. 런치 종료·왼쪽 `reset_alarm`으로 복구. E-stop 한쪽 접점 순간 개방 가설 → `oroha_di_watch`(DI·status 20 Hz 감시, 명령 없음) 추가. 접지 주행 보류.
- 패치 0003(`di` 상태 인터페이스, 한쪽씩 번갈아 PID_DI) + xacro + 실행기 게이트 열림 FAIL + 가드 NOT_TRACKING + 추출 DI 열. 실물에서 DI 0x34·10 Hz 확인(정지).
- **T20261006-08 통과**: E-stop 두 접점 동시 개폐(DI 20 Hz) — 결선 정상.
- **T20261006-09 실패(재현)**: DI 기록 접지 텔레옵 — 게이트는 내내 닫힘(가설 기각). 공통 순서: 바퀴가 명령보다 빨리 돎(곡선 안쪽 끌림·관성 감속) → 그쪽 MD400 출력 0 → 느려져도 수백 ms~3 s 출력 0(속도 루프 와인드업 의심) → 전류 펄스 → OVER_LOAD(이번 오른쪽). MD400 설정 양쪽 같음(램프 0, 게인 15/300/315). 이전 팀 667 rpm/s 소프트 램프 대비 현재 감속 한계 4배.
- **D-16**: diff_cont 가감속 ±0.3 m/s²·±1.0 rad/s², 텔레옵 0.3·q/e 반경 0.6 m (접지 과속 → MD400 출력 0·OVER_LOAD 대책).
- **T20261006-10**: 접지 직진 DONE(0.994 m), 제자리 360° 명령에 실제 267°(스키드 효율 0.74), 원 R0.75 FAIL(안쪽 바퀴 제동 불가). 띄운 감속 기록에서도 제동 전류 없음 → USE_LIMIT_SW 1 방향 게이트 의심.
- **T20261006-11**: 띄운 제동 시험 — USE_LIMIT_SW 1·0 동일, 제동 전류 없음 → MD400은 0이 아닌 명령에서 과속 바퀴를 제동하지 않음. 접지 곡선 불가·OVER_LOAD의 근본 원인.
- **D-17**: 원·S를 내접 다각형(`--sides`, `profiles.poly_arc`)으로 — 끝 자세가 곡선판과 같음, 다각형 S는 R ≤ 0.38(꼭짓점 회전이 반대각을 쓸어 공간 증가), 원 R ≤ 0.75. 테스트 추가.
