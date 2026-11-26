# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Low-priority post-flight worker; never talks to an FC."""

import argparse
import fcntl
import os
from pathlib import Path
import subprocess
from .storage import archive, retain


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--session", type=Path, required=True)
    p.add_argument("--uid", required=True)
    p.add_argument("--level", type=int, default=1)
    p.add_argument("--retain", type=int, default=10)
    a = p.parse_args()
    try:
        os.nice(19)
    except OSError:
        pass
    try:
        subprocess.run(
            ["ionice", "-c", "3", "-p", str(os.getpid())],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        pass
    with (a.root / ".archive.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        print(archive(a.root, a.session, a.uid, a.level), flush=True)
        retain(a.root, a.uid, a.retain)


if __name__ == "__main__":
    main()
