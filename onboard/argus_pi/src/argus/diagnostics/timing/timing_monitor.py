# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Passive timing monitor: FC attitude arrival (fc/attitude on 6370) and 1 s CPU usage. Never sends to FC.

2026-11-16 사용자 지시: 비행·추종 때 함께 기록한다.
- FC 자세: 게이트웨이가 내보내는 ZeroMQ fc/attitude를 구독만 한다(추적기와 같은 PUB). 표본마다
  이 프로세스 수신 시각, 게이트웨이 수신 시각, FC time_boot_ms, 몸체 각속도를 남긴다.
  2026-11-25 batch 6a: 6370 은 팀 [이름][protobuf] 다. 이름이 정확히 fc/attitude 인 메시지만
  src/argus/transport/pb_json.py 로 되살린다 (시작할 때 불러온다). seq 열은 Pi 값(pi_seq) 이다.
- CPU: 1초마다 코어별 사용률, CPU 압력(PSI), 부하, 온도·클럭, ARGUS 프로세스별 CPU%·실행 대기
  (schedstat run delay)·비자발 문맥 전환을 남긴다. 게이트웨이 /state 의 armed 도 함께 남긴다.
  2026-11-16: /state 조회는 5초마다(응답이 무겁고, 게이트웨이 HOLD 안정 판단도 이 조회로 갱신된다).
- 결과: run/timing-monitor/<시작시각>/attitude.csv, cpu.jsonl, monitor.log
- 한 세션 기록이 300 MB를 넘으면 자세 기록만 멈춘다(CPU 기록은 계속). 오래된 기록은 지우지 않는다.
- nice 10으로 실행한다. 실패해도 FC·탐지 기동에는 영향이 없다(run_pi_fc.sh에서 || true).
"""

import argparse
import csv
import fcntl
import json
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

STATE_URL = "http://127.0.0.1:8768/state"
ENDPOINT = "tcp://127.0.0.1:6370"
MAX_SESSION_BYTES = 300 * 1024 * 1024
CLK_TCK = os.sysconf("SC_CLK_TCK")


def read_text(path, default=""):
    try:
        return Path(path).read_text()
    except OSError:
        return default


def cpu_times():
    out = {}
    for line in read_text("/proc/stat").splitlines():
        if line.startswith("cpu"):
            parts = line.split()
            vals = [int(x) for x in parts[1:9]]
            idle = vals[3] + vals[4]
            out[parts[0]] = (sum(vals), idle)
    return out


def psi():
    out = {}
    for line in read_text("/proc/pressure/cpu").splitlines():
        kind, *fields = line.split()
        kv = dict(f.split("=") for f in fields)
        out[kind] = dict(avg10=float(kv["avg10"]), total_us=int(kv["total"]))
    return out


def argus_processes(root):
    """ARGUS 실행 파일(bin/)과 ARGUS 경로를 쓰는 python, mavlink-router."""
    found = {}
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            exe = os.readlink("/proc/%d/exe" % pid)
            args = Path("/proc/%d/cmdline" % pid).read_bytes().split(b"\0")
        except OSError:
            continue
        name = Path(exe).name
        if exe.startswith(str(root / "bin") + "/"):
            label = name
        elif name.startswith("python") and any(str(root).encode() in a for a in args):
            script = next(
                (
                    a.decode("utf8", "replace")
                    for a in args[1:]
                    if a.endswith(b".py") or b"argus." in a
                ),
                "",
            )
            label = "python:" + Path(script).name if script else "python"
        elif name == "mavlink-routerd":
            label = name
        else:
            continue
        found[pid] = label
    return found


def proc_sample(pid):
    stat = read_text("/proc/%d/stat" % pid)
    if not stat:
        return None
    fields = stat[stat.rindex(")") + 2 :].split()
    cpu = int(fields[11]) + int(fields[12])  # utime + stime
    start = int(fields[19])
    run_delay = 0
    nvcsw = 0
    try:
        for task in os.scandir("/proc/%d/task" % pid):
            s = read_text("/proc/%d/task/%s/schedstat" % (pid, task.name)).split()
            if len(s) >= 2:
                run_delay += int(s[1])
            for line in read_text("/proc/%d/task/%s/status" % (pid, task.name)).splitlines():
                if line.startswith("nonvoluntary_ctxt_switches"):
                    nvcsw += int(line.split()[1])
    except OSError:
        pass
    threads = len(os.listdir("/proc/%d/task" % pid)) if os.path.isdir("/proc/%d/task" % pid) else 0
    return dict(cpu_ticks=cpu, start=start, run_delay_ns=run_delay, nvcsw=nvcsw, threads=threads)


def read_state(url):
    try:
        with urllib.request.urlopen(url, timeout=0.5) as r:
            s = json.loads(r.read())
        return dict(
            armed=s.get("armed"), busy=s.get("busy"), mode=s.get("mode") or s.get("flight_mode")
        )
    except Exception:
        return dict(armed=None)


class Session:
    def __init__(self, root):
        self.dir = root / "run/timing-monitor" / time.strftime("%Y%m%d-%H%M%S")
        self.dir.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.bytes = 0
        self.attitude_stopped = False
        self.log_file = (self.dir / "monitor.log").open("a", buffering=1)

    def log(self, text):
        self.log_file.write(time.strftime("%Y-%m-%d %H:%M:%S ") + text + "\n")

    def add(self, n):
        with self.lock:
            self.bytes += n
            return self.bytes


def load_pb_json(root):
    """2026-11-25 batch 6a: 팀 6370 protobuf 해석기 (src/argus/transport/pb_json.py)."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "argus_pb_json", root / "src/argus/transport/pb_json.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def attitude_loop(session, endpoint, stopping, pb_json):
    import zmq

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"fc/attitude")
    sub.setsockopt(zmq.RCVHWM, 1000)
    sub.setsockopt(zmq.LINGER, 0)
    sub.connect(endpoint)
    f = (session.dir / "attitude.csv").open("w", newline="")
    w = csv.writer(f)
    w.writerow(
        [
            "monitor_rx_mono_ns",
            "unix_s",
            "gateway_rx_mono_ns",
            "fc_time_boot_ms",
            "seq",
            "session_id",
            "roll_rad",
            "pitch_rad",
            "yaw_rad",
            "roll_rad_s",
            "pitch_rad_s",
            "yaw_rad_s",
        ]
    )
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    last_flush = time.monotonic()
    count = 0
    while not stopping:
        if not poller.poll(500):
            if time.monotonic() - last_flush > 1:
                f.flush()
                last_flush = time.monotonic()
            continue
        frames = sub.recv_multipart()
        rx = time.monotonic_ns()
        if len(frames) != 2 or frames[0] != b"fc/attitude" or session.attitude_stopped:
            continue
        try:
            m = pb_json.decode("fc/attitude", frames[1])[0]
        except Exception:
            continue
        row = [
            rx,
            "%.3f" % time.time(),
            m.get("received_monotonic_ns"),
            m.get("fc_time_boot_ms"),
            m.get("seq"),
            m.get("session_id"),
            m.get("roll_rad"),
            m.get("pitch_rad"),
            m.get("yaw_rad"),
            m.get("roll_rad_s"),
            m.get("pitch_rad_s"),
            m.get("yaw_rad_s"),
        ]
        w.writerow(row)
        count += 1
        if session.add(120) > MAX_SESSION_BYTES:
            session.attitude_stopped = True
            session.log("기록 300 MB 초과 → 자세 기록 중지 (CPU 기록 계속)")
        if time.monotonic() - last_flush > 1:
            f.flush()
            last_flush = time.monotonic()
    f.close()
    session.log("자세 기록 종료 · 표본 %d개" % count)


def cpu_loop(session, root, url, stopping, period=1.0):
    f = (session.dir / "cpu.jsonl").open("a")
    prev_cpu = cpu_times()
    prev_psi = psi()
    prev_proc = {}
    labels = {}
    next_scan = 0
    state = dict(armed=None)
    next_state = 0
    t_prev = time.monotonic()
    next_t = t_prev + period
    while not stopping:
        time.sleep(max(0, next_t - time.monotonic()))
        next_t += period
        now = time.monotonic()
        dt = now - t_prev
        t_prev = now
        if now >= next_scan:
            labels = argus_processes(root)
            next_scan = now + 5
        cur = cpu_times()
        cores = {}
        for k, (tot, idle) in cur.items():
            if k in prev_cpu:
                dtot = tot - prev_cpu[k][0]
                cores[k] = (
                    round(100 * (1 - (idle - prev_cpu[k][1]) / dtot), 1) if dtot > 0 else None
                )
        prev_cpu = cur
        p = psi()
        pressure = {
            k: dict(
                avg10=v["avg10"],
                stall_ms=round((v["total_us"] - prev_psi.get(k, v)["total_us"]) / 1000, 2),
            )
            for k, v in p.items()
        }
        prev_psi = p
        procs = {}
        for pid, label in list(labels.items()):
            s = proc_sample(pid)
            if s is None:
                labels.pop(pid, None)
                prev_proc.pop(pid, None)
                continue
            old = prev_proc.get(pid)
            if old and old["start"] == s["start"]:
                procs["%s:%d" % (label, pid)] = dict(
                    cpu_pct=round(100 * (s["cpu_ticks"] - old["cpu_ticks"]) / CLK_TCK / dt, 1),
                    run_delay_ms=round((s["run_delay_ns"] - old["run_delay_ns"]) / 1e6, 2),
                    nvcsw=s["nvcsw"] - old["nvcsw"],
                    threads=s["threads"],
                )
            prev_proc[pid] = s
        freqs = []
        for c in range(os.cpu_count() or 4):
            v = read_text("/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq" % c).strip()
            freqs.append(int(v) // 1000 if v.isdigit() else None)
        if now >= next_state:
            state = read_state(url)
            next_state = now + 5
        temp = read_text("/sys/class/thermal/thermal_zone0/temp").strip()
        rec = dict(
            unix_s=round(time.time(), 3),
            mono_ns=time.monotonic_ns(),
            dt_s=round(dt, 3),
            cores=cores,
            psi=pressure,
            loadavg=read_text("/proc/loadavg").split()[:3],
            temp_c=int(temp) / 1000 if temp.lstrip("-").isdigit() else None,
            freq_mhz=freqs,
            state=state,
            procs=procs,
        )
        line = json.dumps(rec, separators=(",", ":")) + "\n"
        f.write(line)
        f.flush()
        session.add(len(line))
    f.close()


def run(root, url, endpoint):
    run_dir = root / "run/timing-monitor"
    run_dir.mkdir(parents=True, exist_ok=True)
    lock = (run_dir / "monitor.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("timing-monitor already running", file=sys.stderr)
        return 0
    try:
        os.nice(10)
    except OSError:
        pass
    pb_json = load_pb_json(root)  # python3-protobuf 가 없으면 기록을 시작하기 전에 멈춘다
    session = Session(root)
    stopping = []
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(1))
    signal.signal(signal.SIGINT, lambda *_: stopping.append(1))
    session.log("시작 pid=%d · %s 구독 · CPU 1초 · 기록 %s" % (os.getpid(), endpoint, session.dir))
    t = threading.Thread(
        target=attitude_loop,
        args=(session, endpoint, stopping, pb_json),
        daemon=True,
        name="attitude",
    )
    t.start()
    try:
        cpu_loop(session, root, url, stopping)
    finally:
        stopping.append(1)
        t.join(timeout=2)
        session.log("종료")
    return 0


def ensure(root, url, endpoint):
    run_dir = root / "run/timing-monitor"
    run_dir.mkdir(parents=True, exist_ok=True)
    with (run_dir / "monitor.lock").open("a") as probe:
        try:
            fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return dict(reused=True)
        fcntl.flock(probe, fcntl.LOCK_UN)
    with (run_dir / "service.log").open("ab") as log:
        child = subprocess.Popen(
            [
                sys.executable,
                "-B",
                str(Path(__file__).resolve()),
                "--root",
                str(root),
                "--state-url",
                url,
                "--endpoint",
                endpoint,
            ],
            cwd=root,
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
        "--ensure", action="store_true", help="start one background monitor if none is running"
    )
    p.add_argument("--state-url", default=STATE_URL)
    p.add_argument("--endpoint", default=ENDPOINT)
    a = p.parse_args()
    root = a.root.resolve()
    if a.ensure:
        print(json.dumps(ensure(root, a.state_url, a.endpoint)))
        return 0
    return run(root, a.state_url, a.endpoint)


if __name__ == "__main__":
    sys.exit(main())
