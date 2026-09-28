"""port_holders finds another process that has the device open."""

import subprocess
import sys
import time

from oroha_power.port_guard import port_holders, require_free

import pytest


def test_finds_other_holder_and_require_free(tmp_path):
    dev = tmp_path / "fake_tty"
    dev.write_text("")
    assert port_holders(str(dev)) == []
    child = subprocess.Popen([sys.executable, "-c",
                              f"import time; f = open({str(dev)!r}); time.sleep(10)"])
    try:
        for _ in range(50):
            if port_holders(str(dev)):
                break
            time.sleep(0.05)
        holders = port_holders(str(dev))
        assert [pid for pid, _ in holders] == [child.pid]
        with pytest.raises(SystemExit):
            require_free(str(dev), "test")
        require_free(str(dev), "test", force=True)
    finally:
        child.kill()
        child.wait()


def test_own_process_is_ignored(tmp_path):
    dev = tmp_path / "fake_tty"
    dev.write_text("")
    with open(dev):
        assert port_holders(str(dev)) == []
