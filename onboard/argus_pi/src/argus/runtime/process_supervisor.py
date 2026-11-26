# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Shared owned-process inspection and spawning; no automatic device actions."""

import os
from pathlib import Path
import socket
import subprocess


def listening(port):
    with socket.socket() as connection:
        connection.settimeout(0.2)
        return connection.connect_ex(("127.0.0.1", port)) == 0


def matching_process(executable):
    expected = executable.resolve()
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            if (process / "exe").resolve() == expected:
                return int(process.name)
        except (OSError, RuntimeError):
            continue
    return None


def owns_listeners(pid, ports):
    """A matching executable alone cannot authenticate someone else's sockets."""
    try:
        sockets = set()
        for fd in (Path("/proc") / str(pid) / "fd").iterdir():
            try:
                sockets.add(os.readlink(fd))
            except FileNotFoundError:
                # The directory iterator's descriptor, or a transient socket, may close.
                continue
        found = set()
        for table in ("tcp", "tcp6"):
            for line in (Path("/proc/net") / table).read_text().splitlines()[1:]:
                fields = line.split()
                if fields[3] == "0A" and "socket:[" + fields[9] + "]" in sockets:
                    found.add(int(fields[1].rsplit(":", 1)[1], 16))
        return set(ports).issubset(found)
    except (OSError, IndexError, ValueError):
        return False


def start(root, run, name, arguments):
    log_path = run / (name + ".log")
    previous_bytes = log_path.stat().st_size if log_path.exists() else 0
    with (run / (name + ".log")).open("ab", buffering=0) as log:
        process = subprocess.Popen(
            arguments,
            cwd=root,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
    process.preview_log_start = previous_bytes
    (run / (name + ".pid")).write_text(str(process.pid), encoding="ascii")
    return process
