"""Fault injection (T20261006-04): 0xFF bytes on the RS485 bus for N s. 0xFF = slave id 255,
never one of ours (1, 2), so no byte run can form a valid request to the MD400s."""
import sys, time, serial
sec = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
s = serial.Serial("/dev/oroha_md400", 19200)
t0 = time.monotonic(); n = 0
while time.monotonic() - t0 < sec:
    n += s.write(b"\xff" * 64); s.flush()
s.close()
print("injected %d bytes of 0xFF in %.1f s" % (n, sec))
