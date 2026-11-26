# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Pause developer tools (VS Code server, rsync, compilers) while the FC is armed; never touches FC.

2026-11-12 사용자 요청: arm 동안 Pi의 개발 도구가 CPU·Wi-Fi를 쓰지 않게 한다.
- 끄지 않고 일시정지(SIGSTOP)했다가 disarm 때 이어서 실행(SIGCONT)한다. 하던 작업을 잃지 않는다.
- sshd와 ARGUS 실행 프로세스(그 조상 포함)는 건드리지 않는다. GUI들이 SSH로 Pi에 붙는다.
- ARGUS 프로세스가 터미널(pty)에 출력하고 있으면 VS Code는 멈추지 않는다. VS Code 터미널을
  멈추면 그 터미널에 쓰던 ARGUS 프로세스가 출력에서 막힐 수 있다.
- arm 판단은 게이트웨이 상태(127.0.0.1:8768/state의 armed)만 읽는다. FC에 아무것도 보내지 않는다.
- 멈춘 목록은 run/flight-quiet/paused.json에 남겨, 이 감시기가 강제 종료돼도 다음 기동 때 푼다.
- 게이트웨이 상태를 오래(기본 120초) 못 읽으면 멈춘 도구를 푼다(영원히 멈춰 있지 않게).
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request

STATE_URL = "http://127.0.0.1:8768/state"
# 일시정지 대상 실행 파일 이름 (경로 안의 /.vscode-server/는 따로 본다)
DEV_NAMES = frozenset(
    (
        "rsync",
        "cc1",
        "cc1plus",
        "as",
        "ld",
        "ld.bfd",
        "ld.gold",
        "collect2",
        "make",
        "gmake",
        "ninja",
        "cmake",
        "ctest",
        "clang-tidy",
        "clang-format",
        "git",
    )
)
NEVER_NAMES = frozenset(("sshd", "sshd-session", "systemd", "bash", "sh", "dbus-daemon"))


def read_proc(pid):
    base = Path("/proc") / str(pid)
    stat = (base / "stat").read_text(encoding="ascii", errors="replace")
    rest = stat[stat.rfind(")") + 2 :].split()
    try:
        exe = os.readlink(base / "exe")
    except OSError:
        exe = ""
    return dict(
        pid=pid,
        state=rest[0],
        ppid=int(rest[1]),
        ticks=int(rest[19]),
        exe=exe,
        args=(base / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace"),
        uid=base.stat().st_uid,
    )


def ttys(pid):
    out = set()
    for fd in (0, 1, 2):
        try:
            target = os.readlink("/proc/%d/fd/%d" % (pid, fd))
        except OSError:
            continue
        if target.startswith("/dev/pts/"):
            out.add(target)
    return out


def user_processes(uid):
    procs = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            p = read_proc(int(entry.name))
        except (OSError, ValueError, IndexError):
            continue
        if p["uid"] == uid and p["state"] != "Z":
            procs[p["pid"]] = p
    return procs


def is_argus_runtime(p, root):
    exe = p["exe"]
    if exe.startswith(str(root / "bin") + "/") or Path(exe).name in (
        "argus-onboard",
        "argus_onboard",
    ):
        return True
    return (
        Path(exe).name.startswith("python")
        and str(root) in p["args"]
        and "/.vscode-server/" not in exe
    )


def select_targets(procs, root, me, names=DEV_NAMES, vscode_too=True, only_pids=None):
    """Return ({pid: proc} to pause, note). Pure function over a process table (tests use fakes).

    only_pids: tests limit pausing to their own children so real tools on the Pi are never stopped.
    """
    runtime = {pid for pid, p in procs.items() if is_argus_runtime(p, root)}
    protected = set(runtime) | {me, 1}
    for pid in runtime:  # ARGUS 프로세스의 조상(띄운 셸·터미널)도 보호
        seen = 0
        parent = procs[pid]["ppid"]
        while parent in procs and seen < 64:
            protected.add(parent)
            parent = procs[parent]["ppid"]
            seen += 1
    runtime_on_tty = any(p.get("ttys") for pid, p in procs.items() if pid in runtime)
    targets = {}
    for pid, p in procs.items():
        if pid in protected or (only_pids is not None and pid not in only_pids):
            continue
        name = Path(p["exe"]).name if p["exe"] else ""
        if name in NEVER_NAMES:
            continue
        vscode = vscode_too and "/.vscode-server/" in p["exe"]
        if vscode and runtime_on_tty:
            continue
        if vscode or name in names:
            targets[pid] = p
    note = "ARGUS 프로세스가 터미널에 출력 중이라 VS Code는 멈추지 않음" if runtime_on_tty else ""
    return targets, note


def alive(pid, ticks):
    try:
        return read_proc(pid)["ticks"] == ticks
    except (OSError, ValueError, IndexError):
        return False


class Guard:
    def __init__(
        self,
        root,
        state_path,
        log,
        names=DEV_NAMES,
        vscode_too=True,
        only_pids=None,
        lost_resume_s=120.0,
    ):
        self.root = root
        self.state_path = state_path
        self.log = log
        self.names = frozenset(names)
        self.vscode_too = vscode_too
        self.only_pids = only_pids
        self.lost_resume_s = lost_resume_s
        self.paused = {}  # pid -> {ticks, name}
        self.last_known = time.monotonic()
        self.last_note = ""
        self._load()

    def _load(self):
        try:
            rows = json.loads(self.state_path.read_text(encoding="utf-8"))["paused"]
        except (OSError, ValueError, KeyError, TypeError):
            return
        for row in rows:
            if alive(int(row["pid"]), int(row["ticks"])):
                self.paused[int(row["pid"])] = dict(
                    ticks=int(row["ticks"]), name=row.get("name", "")
                )
        if self.paused:
            self.log("이전 감시기가 멈춘 %d개를 이어받음" % len(self.paused))

    def _save(self):
        rows = [
            dict(pid=pid, ticks=v["ticks"], name=v["name"])
            for pid, v in sorted(self.paused.items())
        ]
        tmp = self.state_path.with_name(self.state_path.name + ".tmp")
        tmp.write_text(
            json.dumps(dict(paused=rows, updated_unix=time.time()), ensure_ascii=False),
            encoding="utf-8",
        )
        os.replace(tmp, self.state_path)

    def pause_new(self):
        procs = user_processes(os.getuid())
        for pid, p in procs.items():
            p["ttys"] = ttys(pid) if is_argus_runtime(p, self.root) else set()
        targets, note = select_targets(
            procs, self.root, os.getpid(), self.names, self.vscode_too, self.only_pids
        )
        if note and note != self.last_note:
            self.log(note)
        self.last_note = note
        done = []
        for pid, p in targets.items():
            if pid in self.paused:
                continue
            name = Path(p["exe"]).name
            # 먼저 기록하고 멈춘다: 그 사이 감시기가 죽어도 다음 기동 때 풀 수 있다.
            self.paused[pid] = dict(ticks=p["ticks"], name=name)
            self._save()
            try:
                os.kill(pid, signal.SIGSTOP)
                done.append("%s(%d)" % (name, pid))
            except OSError:
                self.paused.pop(pid, None)
                self._save()
        if done:
            self.log("일시정지: " + ", ".join(done))

    def resume_all(self, why):
        if not self.paused:
            return
        done = []
        for pid, v in sorted(self.paused.items()):
            if alive(pid, v["ticks"]):
                try:
                    os.kill(pid, signal.SIGCONT)
                    done.append("%s(%d)" % (v["name"], pid))
                except OSError:
                    pass
        self.paused.clear()
        self._save()
        self.log("재개(%s): %s" % (why, ", ".join(done) if done else "살아 있는 대상 없음"))

    def step(self, armed, now=None):
        """armed: True/False from the gateway, None when unknown (FC or gateway not answering)."""
        now = time.monotonic() if now is None else now
        if armed is True:
            self.last_known = now
            self.pause_new()
        elif armed is False:
            self.last_known = now
            self.resume_all("disarm")
        elif self.paused and now - self.last_known > self.lost_resume_s:
            self.resume_all("arm 상태를 %.0f초 못 읽음" % self.lost_resume_s)


def read_armed(url):
    try:
        with urllib.request.urlopen(url, timeout=1.0) as response:
            state = json.load(response)
    except (OSError, ValueError):
        return None
    if state.get("app") != "argus-real-fc-gui":
        return None
    armed = state.get("armed")
    return armed if isinstance(armed, bool) else None


def run(root, url, period, lost_resume_s):
    run_dir = root / "run/flight-quiet"
    run_dir.mkdir(parents=True, exist_ok=True)
    lock = (run_dir / "guard.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("flight-quiet already running", file=sys.stderr)
        return 0

    def log(text):
        print(time.strftime("%Y-%m-%d %H:%M:%S ") + text, flush=True)

    guard = Guard(root, run_dir / "paused.json", log, lost_resume_s=lost_resume_s)
    stopping = []
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(1))
    signal.signal(signal.SIGINT, lambda *_: stopping.append(1))
    log("시작 pid=%d · arm 동안 개발 도구 일시정지" % os.getpid())
    status = run_dir / "status.json"
    while not stopping:
        armed = read_armed(url)
        guard.step(armed)
        tmp = status.with_name("status.json.tmp")
        tmp.write_text(
            json.dumps(
                dict(
                    pid=os.getpid(), armed=armed, paused=len(guard.paused), updated_unix=time.time()
                )
            ),
            encoding="utf-8",
        )
        os.replace(tmp, status)
        time.sleep(period)
    guard.resume_all("감시기 종료")
    log("종료")
    return 0


def ensure(root):
    run_dir = root / "run/flight-quiet"
    run_dir.mkdir(parents=True, exist_ok=True)
    with (run_dir / "guard.lock").open("a") as probe:
        try:
            fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return dict(reused=True)
        fcntl.flock(probe, fcntl.LOCK_UN)
    env = dict(
        os.environ, PYTHONPATH=str(root / "src") + os.pathsep + os.environ.get("PYTHONPATH", "")
    )
    with (run_dir / "service.log").open("ab") as log:
        child = subprocess.Popen(
            [sys.executable, "-B", "-m", "argus.runtime.flight_quiet", "--root", str(root)],
            cwd=root,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
    return dict(reused=False, pid=child.pid)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--root", type=Path, required=True)
    p.add_argument(
        "--ensure", action="store_true", help="start one background guard if none is running"
    )
    p.add_argument("--state-url", default=STATE_URL)
    p.add_argument("--period", type=float, default=1.0)
    p.add_argument("--lost-resume-s", type=float, default=120.0)
    a = p.parse_args()
    root = a.root.resolve()
    if a.ensure:
        print(json.dumps(ensure(root)))
        return 0
    return run(root, a.state_url, a.period, a.lost_resume_s)


if __name__ == "__main__":
    sys.exit(main())
