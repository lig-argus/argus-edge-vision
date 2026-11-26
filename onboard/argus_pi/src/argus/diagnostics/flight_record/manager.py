# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Start one passive recorder; does not start cameras or FC connections."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from .daemon import load_config


def ensure(root):
    cfg = load_config(root)
    if not cfg["enabled"]:
        return {"enabled": False}
    run = root / "run/flight-recorder"
    run.mkdir(parents=True, exist_ok=True)
    with (run / "start.lock").open("a") as startup:
        deadline = time.monotonic() + 0.5
        while True:
            try:
                fcntl.flock(startup, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise RuntimeError("another flight recorder startup is in progress")
                time.sleep(0.025)
        with (run / "recorder.lock").open("a") as owned:
            try:
                fcntl.flock(owned, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return {"enabled": True, "reused": True}
            fcntl.flock(owned, fcntl.LOCK_UN)
        env = dict(
            os.environ, PYTHONPATH=str(root / "src") + os.pathsep + os.environ.get("PYTHONPATH", "")
        )
        with (run / "service.log").open("ab") as log:
            child = subprocess.Popen(
                [
                    sys.executable,
                    "-B",
                    "-m",
                    "argus.diagnostics.flight_record.daemon",
                    "--root",
                    str(root),
                ],
                cwd=root,
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=log,
                start_new_session=True,
            )
        for _ in range(20):
            if child.poll() is not None:
                raise RuntimeError("flight recorder failed; see run/flight-recorder/service.log")
            try:
                status = json.loads((run / "status.json").read_text())
                if status.get("pid") == child.pid and status.get("enabled"):
                    return {"enabled": True, "pid": child.pid, "reused": False}
            except (OSError, ValueError):
                pass
            time.sleep(0.05)
        raise RuntimeError("flight recorder startup did not become ready")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--ensure", action="store_true")
    a = p.parse_args()
    print(json.dumps(ensure(a.root.resolve())))


if __name__ == "__main__":
    main()
