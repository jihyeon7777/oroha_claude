# 좌표계 (frames)

REP-103: x 전방, y 좌측, z 상방, 각도는 rad, yaw는 z축 반시계(+).

| frame | 정의 | 출처 | 상태 |
|---|---|---|---|
| `odom` | 런치 시점의 `base_footprint` 위치를 원점으로 하는 휠 오도메트리 좌표계 (`diff_cont`, 무슬립 가정) | diff_drive_controller | 확정 |
| `base_footprint` | 네 바퀴 접지점의 중심, 지면 높이, x = 진행 방향 | `oroha_description` | 확정(원점 정의). 휠베이스 실측 후 시각 모델 갱신 |
| `base_link` | `base_footprint` 위 구름반경 0.1258 m(차축 높이). `diff_cont`의 `base_frame_id` | `oroha_description` | 확정 |
| `left_wheels` / `right_wheels` | 좌·우 구동 조인트 `motor_L`/`motor_R`(축 +y)에 달린 링크. 한 쪽 두 바퀴는 벨트로 묶여 하나의 조인트 | `oroha_description` | 확정 |
| `imu_link` | UM7 장착 위치·자세. um7_driver는 NED body→ENU 변환을 적용해 `frame_id: imu_link`로 발행 | `oroha_description` `imu_joint` | **PLACEHOLDER** — 2단계에서 장착 후 축 시험(앞 숙임→pitch, CCW→+gz)으로 rpy 확정 |
| `marker_link` | GT 카메라용 마커의 기준점 | `oroha_description` `marker_joint` | **PLACEHOLDER** — GT 프로그램과 함께 정의 |
| `world` (GT) | 노트북 GT 프로그램의 세계 좌표계(3 m×3 m 실내). `odom`과의 관계는 사후 정렬(시작 자세 기준) | GT 측 | 미정 |

## GT 사후 결합에 필요한 것

- 같은 실험 ID: 로봇 측 `run_id`(`R<YYYYMMDD>-<HHMMSS>-<path>`)를 GT 기록에도 적는다. `/oroha_experiment/event`(transient_local)의 START/END 시각(ROS ns, Pi 시계)이 구간 경계다.
- 변환 사슬: `world` ← (GT가 관측한 `marker_link`) ← `base_link` (`marker_joint`, 확정 예정) ← `odom`(정렬 대상).
- 시간 정렬: [time_sync.md](time_sync.md).
