# 다음 세션 시작점

갱신 2026-10-08 KST. 계획 원문: `/home/oroha/.claude/plans/agile-enchanting-shell.md`(09-28 재검토판). 이 파일은 세션이 끝날 때마다 덮어쓴다.

## 현재 상태

| 항목 | 상태 |
|---|---|
| 빌드 | v1.4.0 + 패치 `0001`(status·read_seq) · `0002`(즉시 재시도) · `0003`(DI) |
| 장소·조건 | 넓은 방(크기 미측정, 5 m 가정), 미끄러운 바닥, 약 50 kg, 10 psi (D-19) |
| 주행 설정 | `wheel_radius 0.003580`(D-15), `wheel_separation 0.631` 유효 트랙(D-18), 가감속 0.3 m/s²·1.0 rad/s²(D-16), 원·S 다각형(D-17), 구간 시작 지연 보상 0.035/0.091 s(D-20) |
| IMU | 장착됨, **읽기만**(D-19). 축 x 앞·y 왼·z 위(T20261008-04). 바이어스는 실행별 rest_pre 차감 |
| 실험 데이터 | ✅ 접지 본편 14 실행(T20261008-01·03, usable) — 직진 4·제자리 4·다각형 원 2·사각 2·다각형 S 2. 줄자 GT 없음(사용자 결정) |
| 논문 묶음 | ✅ `paper_export/20261008_162817`(468 파일, bag 포함, 검증 OK). git 미포함 |
| 키보드 주행 | `bash setup/drive.sh [--record] [--imu]` (곡선 q/e 권장 안 함) |

## 남은 일

1. **백업**: git은 GitHub `jihyeon7777/oroha_claude`(public, `origin`)에 push 완료. 녹화·논문 묶음 압축은 `data/backup/20261008/` — **GitHub 릴리스 업로드는 사용자가 실행**(`setup/release_upload.py`, `records/backup.md`). 이후 커밋은 `git push`로 올린다.
2. (선택) 반복 수 늘리기(경로별 3~5회), 속도 다르게(0.1/0.3 m/s), 공기압 변경 시 B4·D-18·D-20 재측정.
3. (선택) 바퀴별 가드 FAIL 실물 확인(띄운 상태에서 한쪽 손으로 잡기), MDROBOT에 MD400 회생 제동 설정 문의, DMM 버스 전압 1점, 외부 GT(카메라/모캡) 연동.
4. 세션 끝마다: `oroha_ledger --check`, `paper_summary.md` 갱신, 필요 시 `oroha_paper_export --runs usable` 재생성.
