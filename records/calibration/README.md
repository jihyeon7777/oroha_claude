# 교정값

| calib_id | 대상 | 파일 | 상태 |
|---|---|---|---|
| `sensing-20260828` | ACS37030 전류 ×2, 버스전압 분압 (Pi 호스트) | `sensing-20260828.yaml` → `src/oroha_power/config/calibration/`(단일 출처, 심볼릭 링크) | 확정, 0~1.2 A 범위 |
| `wheel-20260909` | 바퀴·구동계 상수 | `wheel-20260909.yaml` | **대체됨**(무적재·공기압 미상, 0.7613 mm/count) |
| `wheel-20261006` | 바퀴·구동계 상수 (약 50 kg, 10 psi) | `wheel-20261006.yaml` | **확정**, 0.7497 mm/count ±0.3 %, diff_cont 0.003580 (T20261006-06) |
