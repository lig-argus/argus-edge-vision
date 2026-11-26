# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""flight_quiet: arm 동안 개발 도구 일시정지. FC·게이트웨이 없음, 시험이 띄운 sleep만 멈춘다."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from argus.runtime import flight_quiet as fq  # noqa: E402

ROOT = Path("/home/user/ARGUS")


def proc(pid, ppid, exe, args="", ttys=()):
    return dict(
        pid=pid, ppid=ppid, exe=exe, args=args, ticks=pid * 10, state="S", uid=1000, ttys=set(ttys)
    )


def table(*rows):
    return {r["pid"]: r for r in rows}


def test_selection():
    vs = "/home/user/.vscode-server/cli/servers/x/server/node"
    procs = table(
        proc(10, 1, "/usr/sbin/sshd"),
        proc(11, 10, "/usr/bin/bash"),
        proc(12, 11, str(ROOT / "bin/argus_fc_gateway")),  # ARGUS: 보호
        proc(20, 1, vs),
        proc(21, 20, vs),  # VS Code
        proc(30, 10, "/usr/bin/rsync", "rsync --server"),
        proc(40, 1, "/usr/libexec/gcc/aarch64-linux-gnu/13/cc1plus"),
        proc(
            50, 1, "/usr/bin/python3.12", "python3 -m argus.x --root " + str(ROOT)
        ),  # ARGUS python
        proc(60, 1, "/usr/bin/vim"),  # 대상 아님
    )
    targets, note = fq.select_targets(procs, ROOT, me=999)
    assert set(targets) == {20, 21, 30, 40}, targets
    assert note == ""

    # 게이트웨이를 VS Code 터미널에서 띄웠으면: 조상 VS Code는 보호
    procs[11]["ppid"] = 21
    targets, _ = fq.select_targets(procs, ROOT, me=999)
    assert 21 not in targets and 20 not in targets and 30 in targets, targets

    # ARGUS 프로세스가 pty에 출력 중이면 VS Code 전체를 멈추지 않는다
    procs[11]["ppid"] = 10
    procs[12]["ttys"] = {"/dev/pts/3"}
    targets, note = fq.select_targets(procs, ROOT, me=999)
    assert set(targets) == {30, 40} and note, (targets, note)
    print("PASS selection: sshd/ARGUS/ancestors kept, pty rule keeps VS Code")


def state_of(pid):
    return fq.read_proc(pid)["state"]


def test_pause_resume_and_recovery():
    kids = [subprocess.Popen(["sleep", "30"]) for _ in range(2)]
    pids = {k.pid for k in kids}
    logs = []
    try:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "paused.json"
            guard = fq.Guard(
                ROOT,
                path,
                logs.append,
                names={"sleep"},
                vscode_too=False,
                only_pids=pids,
                lost_resume_s=5,
            )
            guard.step(None, now=0)
            assert all(state_of(p) == "S" for p in pids), "unknown state must not pause"
            guard.step(True, now=1)
            time.sleep(0.1)
            assert all(state_of(p) == "T" for p in pids), [state_of(p) for p in pids]
            assert {r["pid"] for r in json.loads(path.read_text())["paused"]} == pids

            # 감시기가 강제 종료됐다고 보고 새 감시기가 파일에서 이어받아 disarm 때 푼다
            guard2 = fq.Guard(
                ROOT,
                path,
                logs.append,
                names={"sleep"},
                vscode_too=False,
                only_pids=pids,
                lost_resume_s=5,
            )
            assert set(guard2.paused) == pids
            guard2.step(None, now=2)
            assert all(state_of(p) == "T" for p in pids), "short unknown keeps paused"
            guard2.step(False, now=3)
            time.sleep(0.1)
            assert all(state_of(p) == "S" for p in pids), [state_of(p) for p in pids]
            assert json.loads(path.read_text())["paused"] == []

            # arm 뒤 상태를 오래 못 읽으면 푼다
            guard2.step(True, now=10)
            time.sleep(0.1)
            assert all(state_of(p) == "T" for p in pids)
            guard2.step(None, now=14)
            assert all(state_of(p) == "T" for p in pids)
            guard2.step(None, now=16)
            time.sleep(0.1)
            assert all(state_of(p) == "S" for p in pids), "lost state resumes"
    finally:
        for k in kids:
            k.kill()
            k.wait()
    print("PASS pause/resume: arm pauses, disarm resumes, file recovery, lost-state resume")


if __name__ == "__main__":
    test_selection()
    test_pause_resume_and_recovery()
