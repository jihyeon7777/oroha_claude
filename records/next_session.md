# 다음 세션 시작점

갱신 2026-09-29 16:10 KST. 계획 원문: `/home/oroha/.claude/plans/agile-enchanting-shell.md`. 이 파일은 세션이 끝날 때마다 덮어쓴다.

## 현재 상태

| 항목 | 상태 |
|---|---|
| 빌드 | v1.4.0 + `patches/external/mdrobot_motor_driver/0001`(status·status2·read_seq) + `0002`(즉시 재시도·inter_frame_delay). `bash patches/apply_patches.sh` 후 빌드 |
| 세션 A (띄움) | A1~A9 통과: 방향(T20260928-05), E-stop(T20260929-01, `use_limit_sw 1`), ros2_control 주행·정지 행렬(T-02·T-04), 실행기+계측(T-05~07) |
| 통신 두절 | **원인 확인·운용 복구**(T20260929-07): MD400 수신 잠김, 패치 0002로 사각 8/8 DONE. 근본 원인(펌웨어)은 미해결 — 재발 빈도는 로그 `immediate retry ok`로 추적 |
| 남은 세션 A | A10: 데드맨 텔레옵(사용자 터미널, 띄움), UM7 통신 점검(미고정) |
| 로봇 | 접지 주행 이력 없음. 바퀴 띄운 상태 |

## 다음 순서

1. A10 텔레옵(사용자 터미널 `ros2 run oroha_teleop deadman_teleop`, 떼면 ≤0.3 s 감속) · UM7 통신 점검(주기·checksum).
2. 세션 B: B2 전류 의미 분리(H3) → B3 조건 실측(사용자: 수평·질량·공기압·차체 치수) → B4 손밀기(대각선) → B5 텔레옵 접지 → B6 접지 실행기(사용자 터미널 1회씩, v ≤0.25, 방 크기 검사).
3. 접지 실행 시 `joint_diag`(summary.yaml)와 런치 로그의 재시도 횟수를 함께 본다.

## 결정 대기 (해당 세션에서)

UM7 장착 위치·방향 · git 원격과 bag 외부 백업 위치 · DMM Δ/VREF 측정 여부 · (잠김 재발 시) MD400별 RS485 어댑터 분리 또는 제조사 문의.
