# Pico 계측 펌웨어 (MicroPython)

| 파일 | 내용 |
|---|---|
| `main.py` | 현재 펌웨어 **oroha-bench-1.2** (2026-09-22). 1.1 + 유휴 루프에서 `mono_us()` 갱신 — 537 s 이상 스트림 정지 후 `t_us`가 감기던 문제 수정. 표본·프로토콜·상수 변경 없음 |
| `main.py.sha256` | 위 파일의 sha256. **장치의 main.py와 같아야 한다** |
| `archive/main-1.1.py` | 인계 시점 펌웨어(sha256 `9ce752a9…`), 2026-08-28 이후 모든 인계 데이터를 만든 판 |

## 플래시·검증

```bash
source setup/env.sh                         # mpremote 경로
P=/dev/oroha_pico                           # udev 전에는 /dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_*-if00
mpremote connect $P fs cp firmware/pico/main.py :main.py
mpremote connect $P reset
# 장치 사본 해시 확인 (파일 sha256 == firmware/pico/main.py.sha256)
mpremote connect $P exec "import hashlib,binascii; h=hashlib.sha256(); f=open('main.py','rb'); [h.update(c) for c in iter(lambda: f.read(4096), b'')]; print(binascii.hexlify(h.digest()).decode())"
```

REPL(`mpremote connect $P repl`, Ctrl-] 종료)에서 `C`를 보내면 `#CFG fw=oroha-bench-1.2`가 보여야 한다. 명령 표와 `D,` 라인 형식은 `main.py` 상단 docstring과 `oroha_power/protocol.py` 참조.
