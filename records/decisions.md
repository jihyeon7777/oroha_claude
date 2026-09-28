# 설계 결정 (ADR-lite)

| ID | 날짜 | 결정 | 근거 · 대안 |
|---|---|---|---|
| D-01 | 2026-09-22 | 모터 드라이버 기반 = 공개판 v1.4.0 커밋 고정, 외부 소스는 `oroha.repos`+`patches/`로만 관리 | 로컬 bringup 브랜치는 코드 라인이 오래됐고(0.2.4) 공개판은 twin 실물 검증·실측 RPM·전류 상태 제공. 대안: bringup 브랜치 그대로 사용 |
| D-02 | 2026-09-22 | Pico 펌웨어 MicroPython 유지, 환산은 호스트 | 50 Hz 목표에 충분, 재검증 부담 최소. 대안: C SDK 재작성 |
| D-03 | 2026-09-22 | 실험 경로 = 시간 기반 개방루프 속도 프로파일(램프), 오도메트리는 기록만 | 명령 경로가 해석적으로 정의돼 재현·비교가 쉽다. skid-steer 제자리 회전에서 휠 yaw는 불신. 대안: 오도메트리 기반 구간 전환 |
| D-04 | 2026-09-22 | 계측 메시지는 커스텀 `PowerSample`(raw min/max·flags·calib_id 포함) | 계측_GT_연동 §1: 원시 ADC와 환산값·교정 ID를 함께 기록. 표준 Vector3Stamped는 정보 손실 |
| D-05 | 2026-09-22 | OROHA 전용 description·launch(diffbot 예제 패턴), 드라이버의 bringup.launch.py 미사용 | 드라이버 런치는 외부 URDF를 받지 못함. 한 쪽 두 바퀴는 벨트로 묶여 조인트 2개로 모델링 |
| D-06 | 2026-09-22 | 장치 이름 `/dev/oroha_md400`·`/dev/oroha_pico`·`/dev/oroha_um7` (udev serial 고정) | by-id 경로는 길고 어댑터 교체 시 바뀜; 확인표 §5에 by-id 원본을 남김 |
| D-07 | 2026-09-22 | 시간 기준: ROS 시계(시스템 시계, chrony) 하나. Pico는 `t_us`+min-filter 오프셋을 `device_stamp`로, UM7은 수신 시각 | USB CDC 묶음 전송(≈65 ms)이라 정밀 정렬은 `t_us`. Pi가 GT 노트북의 NTP 서버 |
| D-08 | 2026-09-28 | 시험 공간은 3 m×3 m 방만. 접지 경로는 궤적+차체 반대각+여유 0.3 m 검사(벽/대각선 배치) 통과 크기만, 손밀기는 대각선 3회전 | 사용자 결정. 6 m 직진·6 m 손밀기 불가 |
| D-09 | 2026-09-28 | mdrobot 플러그인 수정(on_shutdown 정지, status·read_seq 인터페이스)은 `patches/`로 로컬 적용, 업스트림은 나중에 | 사용자 결정. v1.4.0 고정 유지, 재현 가능 |
| D-10 | 2026-09-28 | 안전 원칙: 모든 모션 명령은 스스로 끝남, 런치 안전망(ros2_control_node 종료 → MD400 정지) + `oroha_md_stop`, E-stop 해제 규칙은 E1 시험으로 확정, Claude는 실시간 정지 경로가 아님 | MD400 통신 워치독 없음, 플러그인 on_shutdown 없음, RS485 분리 시 정지 명령 전달 불가 |
| D-11 | 2026-09-28 | cmd_vel 발행은 RELIABLE, `diff_cont.base_frame_id = base_footprint`, `max_deceleration` 사용, forward_command_controller 미사용 | QoS 호환(RELIABLE→모든 구독), TF 단일 부모, 4.42 파라미터, 타임아웃 없는 컨트롤러 배제 |
