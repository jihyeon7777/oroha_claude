# 다음 세션 시작점

작성 2026-09-23 10:10 KST (배터리 충전을 위한 Pi 종료 직전). 이 파일은 세션이 끝날 때마다 덮어쓴다.

## 종료 직전 확인한 상태 (읽기 전용 점검)

| 항목 | 상태 |
|---|---|
| 시스템 설치 | ✅ `setup/install_system.sh` 실행됨: ros-jazzy-ros2-control **4.48.0**, ros2-controllers 4.42.1, xacro 2.1.1, diagnostic-updater, rqt-robot-monitor, chrony 4.5(활성, timesyncd 비활성), python3-pandas |
| udev | ✅ `/dev/oroha_md400`→ttyUSB0(FTDI BG043HTG), `/dev/oroha_pico`→ttyACM0, `/dev/oroha_um7`→ttyUSB1(CP2102 `…_0001-if00-port0`) |
| 시간 | ✅ Asia/Seoul, chrony NTP 동기(stratum 3, 오프셋 µs). **이전 시험 7건의 시각 표기는 America/New_York(EDT)** — `tests.csv`의 `tz` 열로 구분 |
| Pico | 펌웨어 `oroha-bench-1.2`(sha `bace9505…`), 장치 해시 = 저장소 파일. 노드·preflight 실물 검증 완료 |
| MD400 ×2 | preflight 통과(fw v8.6, 알람 0). **주행은 아직 한 번도 안 했음**(ros2_control 경유 twin 미검증) |
| UM7 | USB 연결됨(`/dev/oroha_um7`), **섀시 고정 안 됨**, 드라이버 미실행 |
| 로봇 | 사용자: 모터 시험 준비(바퀴 띄움·E-stop) 해 둠. 배터리 충전 후 재확인 |
| 빌드 | `oroha_msgs`·`oroha_power`·`oroha_experiment`·`oroha_tools`·`oroha_teleop`·`mdrobot`(py)·`mdrobot_cpp` 빌드됨. **미빌드: `mdrobot_ros2_control`·`oroha_description`·`oroha_bringup`**(설치 전이라) |
| git | 깨끗, HEAD `e230dea`. 잔여 ROS 노드 없음 |

## 다음 세션 순서

### 0단계 관문 (무동작)

1. 부팅 후 `ls /dev/oroha_*`(3개), `timedatectl`(Asia/Seoul), `cd ~/oroha_claude && source setup/env.sh`.
2. **전체 빌드**: `colcon build --symlink-install --packages-up-to oroha_bringup oroha_experiment oroha_tools oroha_teleop` → 관문은 v1.4.0 `mdrobot_ros2_control`(`on_init(HardwareComponentInterfaceParams&)`)이 ros2_control 4.48에서 컴파일되는지. 실패하면 `patches/external/mdrobot_motor_driver/`로 고치고 `apply_patches.sh`에 태운다. → **T-08**.
3. `source setup/env.sh` 다시 → `colcon test --packages-select oroha_experiment oroha_power oroha_tools mdrobot mdrobot_cpp; colcon test-result --verbose`.
4. **mock bringup**: `ros2 launch oroha_bringup robot.launch.py use_mock_hardware:=true power:=false` → 다른 터미널에서 `ros2 control list_controllers`(`joint_state_broadcaster`·`diff_cont` active, `wheel_vel_cont` inactive), `ros2 topic hz /joint_states /diff_cont/odom`, `ros2 run oroha_teleop deadman_teleop`로 odom이 움직이는지, `ros2 launch oroha_bringup view.launch.py`(RViz, 치수는 PLACEHOLDER). → **T-09**. xacro 오류가 나면 `xacro src/oroha_description/urdf/oroha.urdf.xacro use_mock_hardware:=true`로 단독 확인.
5. mock 상태에서 실험 실행기 한 바퀴: `ros2 run oroha_experiment runner --ros-args -p require_power:=false` + `oroha_exp run --path square --side 1.0 --v 0.2 --yes` → odom 열이 채워진 CSV(`oroha_export_csv`)와 `summary.yaml`의 odometry vs ideal 확인. (preflight JSON이 6 h 넘게 오래됐으면 `-p require_preflight:=false` 또는 preflight 재실행.)

### 1단계 — 모터 (바퀴 띄움, E-stop 손에, 모터 버스 ON)

6. `oroha_preflight` (런치 전, RS485 독점) → GO 확인.
7. `ros2 run oroha_tools oroha_direction_check --id 1` → 기대 **오른쪽 바퀴 전진**; `--id 2` → 기대 **왼쪽 바퀴 후진**(거울 장착). 관측을 프롬프트에 답하면 `records/preflight/direction_*.json`에 남는다. → **T-10**, 확인표 §5 'id 1/2 ↔ 좌우, +rpm 방향' 행 갱신. 어긋나면 `oroha_controllers.yaml`의 `motor_id_L/R`·`reverse_*`를 고친다.
8. 실물 런치 `ros2 launch oroha_bringup robot.launch.py` → `ros2 control list_controllers`·`/joint_states`(10 Hz, effort=전류) 확인 →
   - 바퀴별: `ros2 control switch_controllers --deactivate diff_cont --activate wheel_vel_cont` → `ros2 topic pub --once /wheel_vel_cont/commands std_msgs/msg/Float64MultiArray "{data: [10.0, 0.0]}"`(motor_L +10 rad/s → **왼쪽 전진**이어야 함; reverse가 적용된 뒤 값) → `[0.0, 10.0]` → `[0.0, 0.0]` → 다시 `--deactivate wheel_vel_cont --activate diff_cont`.
   - 차체: `ros2 topic pub -r 10 /diff_cont/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.1}}}"` → 양쪽 전진, `angular z 0.5` → 좌(CCW) 회전(왼쪽 후진·오른쪽 전진). → **T-11**.
   - 정지 시험: 발행 중단 → 0.5 s 내 정지(`cmd_vel_timeout`); 주행 중 RS485 USB 분리 → 양쪽 정지 + 로그에 ERROR/torque_off; E-stop → 정지. → **T-12**.
   - 텔레옵(띄운 상태): `ros2 run oroha_teleop deadman_teleop` 데드맨(손 떼면 0.1~0.8 s 안에 감속) 확인 → **T-13**.
9. 접지(공간 확보 후, 고임목·감시자): `oroha_wheel_push --dist 6.000`(토크오프 손밀기, 0.7613 mm/count 대조) → 0.2 m/s 직진 6 m 줄자 대비 odom 오차 → 제자리 360°(4륜 유효 트랙 기록만) → 텔레옵 5분. → **T-14~16**, 확인표 §1에 질량·적재·공기압·휠베이스·차체 치수 실측, `records/calibration/wheel-20260909.yaml` status 갱신.

### 2단계 — 계측·IMU

10. UM7 섀시 고정(위치·방향은 사용자와 결정) → `ros2 launch oroha_bringup robot.launch.py imu:=true` → `ros2 topic hz /imu/data`(~20 Hz 예상), `/diagnostics` 체크섬 0 → 축 시험(앞 숙임→pitch 부호, CCW 회전→+gz) → `oroha_description`의 `imu_joint` rpy 확정 → `ros2 service call /zero_gyros std_srvs/srv/Trigger` → `oroha_um7_static`(미작성, `test/um7_static_test.py` 포팅) 5분 → 확인표 §4, `docs/frames.md`. → **T-17~18**.
11. Pico는 이미 검증됨. 남은 것: 띄운 상태 3000 rpm에서 i_right/i_left vs `/joint_states` effort 비율(**T-19**), 전류 부호(방전 +) 확인.

## 사용자와 정할 것

- UM7 고정 위치·방향(imu_link 원점·자세).
- 접지 시험 장소(3 m×3 m)·노면·바닥 기울기(수평계 1분) — 실행 조건 프롬프트에 들어간다.
- 로봇 질량·적재·타이어 공기압 실측(확인표 §1 미기록 항목).
- 오도메트리 yaw 오차가 크면 `wheel_separation_multiplier`를 기록용으로만 둘지(보정은 범위 밖).

## 작업 규칙 (요약)

- 시험마다 `records/tests/tests.csv` 행 + `T<YYYYMMDD>-<NN>.md`(다음 ID **T20260923-01**부터; 제목에 쉼표가 있으면 따옴표). `oroha_ledger --check`로 점검. `records/changes.md`에 한 줄.
- 노드는 `(setsid ros2 run … > log 2>&1 &)`로, 정리는 `pgrep -f '^/usr/bin/python3 .*[p]attern'` 패턴으로만. 시험 전 `ros2 node list` 중복 확인.
- `use_limit_sw 0` 유지. 모터 시험은 띄운 상태 먼저, E-stop 손에. MD400은 통신 워치독이 없다.
- 미작성 도구: `oroha_um7_static`(2단계), 텔레옵 실물 검증, `records/paper_summary.md` 채우기(5단계).
