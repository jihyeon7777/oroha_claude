"""Who else has this serial port open? (ROS-free; used by oroha_power and oroha_tools)

Neither the mdrobot C++ transport nor pyserial lock the tty, so a second process can
open /dev/oroha_md400 while ros2_control is running and become a second Modbus master,
or send X/Z to the Pico under a running oroha_power. Callers refuse to start when the
port is held (except emergency stop tools, which only warn).
"""

from __future__ import annotations

import os
from typing import List, Tuple


def port_holders(dev: str) -> List[Tuple[int, str]]:
    """[(pid, cmdline)] of other processes of this user that have `dev` open."""
    real = os.path.realpath(dev)
    me = os.getpid()
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit() or int(name) == me:
            continue
        fd_dir = f"/proc/{name}/fd"
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.readlink(os.path.join(fd_dir, fd)) == real:
                    with open(f"/proc/{name}/cmdline", "rb") as f:
                        cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
                    out.append((int(name), cmd))
                    break
            except OSError:
                continue
    return out


def describe(holders: List[Tuple[int, str]]) -> str:
    return "; ".join(f"pid {pid}: {cmd[:120]}" for pid, cmd in holders)


def require_free(dev: str, what: str, force: bool = False) -> None:
    """Raise SystemExit when another process holds `dev` (unless force)."""
    holders = port_holders(dev)
    if holders and not force:
        raise SystemExit(
            f"{what}: {dev} is already open by {describe(holders)}\n"
            f"  stop that process first (a second Modbus master / Pico client corrupts the bus),"
            f" or pass --force if you are sure.")
