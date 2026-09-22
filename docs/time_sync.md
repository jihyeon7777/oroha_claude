# 시간 기준과 동기

| 시계 | 무엇 | 기록 위치 | 정렬 |
|---|---|---|---|
| Pi 시스템 시계 | ROS 시각(`ros_t_ns`, 헤더 stamp의 기준). chrony로 NTP 동기, GT 노트북에는 서버(`setup/chrony/oroha.conf`, `allow 192.168.5.0/24`) | bag 수신 시각, 모든 헤더 | 기준 |
| Pico `t_us` | 펌웨어 단조 µs(1.2부터 유휴 중에도 유지). USB CDC 묶음 전송으로 수신 시각은 ≈65 ms 흔들림 | `PowerSample.t_us`, `device_stamp`(=t_us+오프셋), `sync_offset_s`, `sync_residual_ms` | 최소값 필터(NTP식) 오프셋. 실측 잔차 0.4 ms(T20260922-05) |
| UM7 | 장치 시각 없음. 수신 시각으로 stamp. 한 메시지의 필드가 ≈13 ms 안의 다른 패킷에서 올 수 있음 | `imu/*` 헤더 | 수신 시각 그대로(오차 ≤ 13 ms 가정, 미측정) |
| MD400 | 장치 시각 없음. 10 Hz 폴링 주기의 컨트롤러 갱신 시각 | `/joint_states`, `/diff_cont/odom` 헤더 | 폴링 지연 ≤ 100 ms |
| GT 노트북 | 노트북 시스템 시계. chrony 클라이언트로 Pi에 동기(LAN, 기대 오차 < 1 ms) | GT 기록 | `chronyc tracking`을 양쪽 실행 전후에 기록 |

- 실행마다 `versions.yaml`에 Pi의 `timezone`·`chrony_tracking`이 저장된다. CSV의 `t`는 START 이벤트 기준 초.
- 시간대: 기록은 UTC(`*_utc`), 파일명·run_id는 Pi 로컬 시각(설정 후 Asia/Seoul). 묶음 README에 생성 시각대를 적는다.
- 미검증: GT 노트북과의 실제 정렬 오차(노트북 준비 후 측정), UM7 필드 간 시각 차.
