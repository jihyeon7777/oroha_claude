"""Workspace locations shared by the tools (ROS-free)."""

from __future__ import annotations

import os
from pathlib import Path


def workspace_root() -> Path:
    """$OROHA_WS if set (setup/env.sh), else walk up from cwd looking for oroha.repos."""
    env = os.environ.get("OROHA_WS")
    if env:
        return Path(env)
    p = Path.cwd().resolve()
    for cand in (p, *p.parents):
        if (cand / "oroha.repos").exists():
            return cand
    return p


def records_dir() -> Path:
    return workspace_root() / "records"


def runs_dir() -> Path:
    return workspace_root() / "data" / "runs"
