# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Read-only ARGUS process inventory and guarded SIGTERM; never sends FC commands.

2026-11-10 사용자 확정: restart는 기체 상태를 확인하지 않고 ARGUS IR 프로세스 전부를
선점해 내린 뒤(SIGTERM 3초 → SIGKILL) GUI 연결 때와 같은 방식으로 게이트웨이를 다시 띄운다.
RGB 독립 경로, mavlink-router, 비행 기록기는 건드리지 않는다. FC 명령은 보내지 않는다.
"""

from pathlib import Path
import argparse
import json
import os
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from deployment_config import load

IR_NAMES = {
    "argus-onboard",
    "argus-frame-source",
    "i3_managed_capture",
    "argus_preview_bus",
    "argus_preview_udp_relay",
    "argus_follow_tracker",
    "argus_follow_controller",
}
RGB_NAMES = {"argus_follow_tracker", "argus_follow_controller", "argus_target_diagnostic"}
IR_SCRIPTS = {
    str(ROOT / "src/argus/diagnostics/tracking/ground_recorder.py"),
    str(ROOT / "src/argus/diagnostics/tracking/native_ground_recorder.py"),
    str(ROOT / "src/argus/diagnostics/tracking/maintenance_observer.py"),
}


def start_ticks(proc):
    data = (proc / "stat").read_text(encoding="ascii")
    return int(data[data.rfind(")") + 2 :].split()[19])


def inventory():
    items = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            exe = Path(os.readlink(proc / "exe"))
            args = (proc / "cmdline").read_bytes().split(b"\0")
            kind = ""
            protected = False
            reason = ""
            if exe.parent == ROOT / "bin" and exe.name in IR_NAMES:
                kind = exe.name
            elif exe.parent == Path("/home/user/ARGUS_RGB/ucmc/bin") and exe.name in RGB_NAMES:
                kind = "RGB/" + exe.name
                protected = True
                reason = "RGB 독립 제어 경로는 여기서 종료하지 않습니다"
            elif exe == Path("/opt/argus/bin/argus_onboard"):
                kind = "onboard(메인 관제 연결)"
                protected = True
                reason = "메인 관제 연결을 보존합니다"
            elif exe == Path("/opt/argus/bin/ir_video_sender"):
                kind = "IR 영상 송신기"
            elif any(os.fsencode(path) in args for path in IR_SCRIPTS):
                kind = (
                    "IR 대상 상태 보기"
                    if os.fsencode(
                        str(ROOT / "src/argus/diagnostics/tracking/maintenance_observer.py")
                    )
                    in args
                    else "IR 지상 진단 기록기"
                )
            elif b"/home/user/ARGUS_RGB/live/producer.py" in args:
                kind = "RGB 영상·검출"
                protected = True
                reason = "RGB 독립 실행 경로는 여기서 종료하지 않습니다"
            elif b"/home/user/ARGUS_RGB/ucmc/apps/real_fc/tools/real_fc/gateway.py" in args or (
                b"tools.real_fc.gateway" in args
                and str((proc / "cwd").resolve()).startswith("/home/user/ARGUS_RGB/")
            ):
                kind = "RGB FC 연결부"
                protected = True
                reason = "RGB 독립 FC 연결을 보존합니다"
            elif b"tools.real_fc.gateway" in args and (proc / "cwd").resolve() == ROOT:
                kind = "IR FC 연결부"
                protected = True
                reason = "현재 FC 연결 경로를 보존합니다"
            if not kind:
                continue
            if proc.stat().st_uid != os.getuid():
                protected = True
                reason = "현재 사용자 소유 프로세스가 아닙니다"
            items.append(
                dict(
                    pid=int(proc.name),
                    start_ticks=start_ticks(proc),
                    name=kind,
                    protected=protected,
                    reason=reason,
                )
            )
        except (OSError, ValueError, IndexError):
            continue
    return sorted(items, key=lambda x: (x["name"], x["pid"]))


def load_pb_json():
    """2026-11-25 batch 6a: fc/state (6370) 는 팀 [이름][protobuf] (src/argus/transport/pb_json.py)."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "argus_pb_json", ROOT / "src/argus/transport/pb_json.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def confirmed_ground(config):
    import zmq

    # 2026-11-25 batch 6a: 해석기는 소켓보다 먼저 불러온다. 이름이 정확히 fc/state 가 아니거나
    # 해석하지 못한 본문은 건너뛴다 (지상 확인이 되지 않는다). 키 검사는 되살린 dict 에 그대로 한다:
    # status 가 없는 본문은 해석 실패라 armed, in_air 가 없는 경우와 같이 거부된다.
    pb_json = load_pb_json()
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    try:
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVTIMEO, 1200)
        sock.connect(config["endpoints"]["fc_state"])
        sock.setsockopt(zmq.SUBSCRIBE, b"fc/state")
        until = time.monotonic() + 1.2
        while time.monotonic() < until:
            try:
                parts = sock.recv_multipart()
            except zmq.Again:
                break
            if len(parts) != 2 or parts[0] != b"fc/state" or len(parts[1]) > 1024 * 1024:
                continue
            try:
                data = pb_json.decode("fc/state", parts[1])[0]
            except ValueError:
                continue
            if not isinstance(data, dict):
                continue
            stamp = data.get("received_monotonic_ns")
            now = time.monotonic_ns()
            if (
                data.get("schema") == "argus.fc.state.v1"
                and data.get("clock_domain") == "host_monotonic"
                and data.get("uid") == config["device"]["expected_fc_uid"]
                and data.get("connected") is True
                and data.get("armed") is False
                and data.get("in_air") is False
                and data.get("ground_known") is True
                and data.get("landed") is True
                and data.get("offboard") is False
                and data.get("rc_override") is False
                and type(stamp) is int
                and 0 <= now - stamp <= 500_000_000
            ):
                return True
        return False
    finally:
        sock.close()
        ctx.term()


def stop(pid, ticks):
    import fcntl

    config = load(ROOT)
    run = ROOT / "run/follow_b"
    if not run.is_dir():
        raise ValueError("추종 준비 잠금 경로 없음")
    with (run / "prepare.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        entries = inventory()
        match = [x for x in entries if x["pid"] == pid and x["start_ticks"] == ticks]
        if len(match) != 1:
            raise ValueError("프로세스가 종료됐거나 변경됐습니다")
        item = match[0]
        if item["protected"]:
            raise ValueError(item["reason"])
        control_present = any(
            x["name"] in ("argus_follow_controller", "onboard(메인 관제 연결)", "IR FC 연결부")
            or x["name"] in ("RGB/argus_follow_controller", "RGB FC 연결부")
            for x in entries
        )
        if control_present and not confirmed_ground(config):
            raise ValueError("FC 신원·신선한 지상·무장 해제 상태를 확인할 수 없습니다")
        if any(x["name"] in ("RGB/argus_follow_controller", "RGB FC 연결부") for x in entries):
            raise ValueError("RGB 추종 프로세스의 상태를 이 경로에서 확인할 수 없습니다")
        if any(x["name"] == "argus_follow_controller" for x in entries):
            sys.path.insert(0, str(ROOT / "apps/real_fc"))
            from tools.real_fc.follow_proxy import forward_vmy, validate_status

            status = validate_status(
                forward_vmy(
                    dict(action="status", expected_uid=config["device"]["expected_fc_uid"]),
                    config["device"]["expected_fc_uid"],
                    timeout_ms=400,
                ),
                config["device"]["expected_fc_uid"],
            )
            if status.get("active") is not False or status.get("bench_active") is not False:
                raise ValueError("추종 또는 지상 yaw 시험 중에는 종료할 수 없습니다")
        if item["name"] == "IR FC 연결부":
            import urllib.request

            with urllib.request.urlopen("http://127.0.0.1:8768/state", timeout=0.6) as response:
                gateway = json.load(response)
            owner = gateway.get("follow_b") or {}
            if not (
                gateway.get("app") == "argus-real-fc-gui"
                and gateway.get("expected_uid") == config["device"]["expected_fc_uid"]
                and gateway.get("fresh") is True
                and gateway.get("identity_ok") is True
                and gateway.get("armed") is False
                and gateway.get("ground") is True
                and not gateway.get("busy")
                and not gateway.get("external")
                and not gateway.get("owned_flight")
                and not owner.get("owner_active")
                and not owner.get("bench_active")
            ):
                raise ValueError("FC 연결부의 지상·무장 해제·송신권 상태 미확인")
        # Re-read identity immediately before SIGTERM to guard against PID reuse.
        match = [x for x in inventory() if x["pid"] == pid and x["start_ticks"] == ticks]
        if len(match) != 1 or match[0]["protected"]:
            raise ValueError("프로세스 식별이 변경됐습니다")
        os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if not any(x["pid"] == pid and x["start_ticks"] == ticks for x in inventory()):
                return dict(accepted=True, reason="프로세스 종료 확인", process=item)
            time.sleep(0.1)
        return dict(accepted=True, reason="종료 요청됨 · 아직 종료 확인 전", process=item)


RESTART_STATUS = ROOT / "run/connection/restart-all.json"
RESTART_LOG = ROOT / "run/connection/restart-all.log"
# 재시작 대상에서 빼는 명령행 표시: RGB 독립 경로, 비행 기록기, 이 관리 스크립트 자신.
# 2026-11-12 flight_quiet: 멈춘 개발 도구를 기억하는 감시기라 전체 재시작에서도 내리지 않는다.
RESTART_EXCLUDE = (b"ARGUS_RGB", b"flight_recorder", b"maintenance_processes.py", b"flight_quiet")
RESTART_WAIT = (
    "argus_fc_gateway",
    "argus_follow_controller",
    "argus_follow_tracker",
    "argus_main_gcs_video",
)


def write_restart(**values):
    RESTART_STATUS.parent.mkdir(parents=True, exist_ok=True)
    tmp = RESTART_STATUS.with_name(RESTART_STATUS.name + ".tmp")
    tmp.write_text(
        json.dumps(dict(values, updated_unix=time.time()), ensure_ascii=False), encoding="utf-8"
    )
    os.replace(tmp, RESTART_STATUS)


def restart_status():
    try:
        return json.loads(RESTART_STATUS.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def user_processes():
    me = os.getuid()
    skip = {os.getpid(), os.getppid()}
    procs = {}
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) in skip:
            continue
        try:
            if proc.stat().st_uid != me:
                continue
            stat = (proc / "stat").read_text(encoding="ascii")
            rest = stat[stat.rfind(")") + 2 :].split()
            if rest[0] == "Z":
                continue
            procs[int(proc.name)] = dict(
                ppid=int(rest[1]),
                ticks=int(rest[19]),
                exe=Path(os.readlink(proc / "exe")),
                args=(proc / "cmdline").read_bytes(),
                cwd=Path(os.readlink(proc / "cwd")),
            )
        except (OSError, ValueError, IndexError):
            continue
    return procs


def restart_targets():
    procs = user_processes()
    roots = set()
    for pid, p in procs.items():
        if any(x in p["args"] for x in RESTART_EXCLUDE) or "ARGUS_RGB" in str(p["exe"]):
            continue
        if p["exe"].name == "mavlink-routerd":
            continue
        if p["exe"].parent == ROOT / "bin" or p["exe"].name in ("argus-onboard", "argus_onboard"):
            roots.add(pid)
        elif (
            p["cwd"] == ROOT
            and p["exe"].name.startswith("python")
            and os.fsencode(str(ROOT)) in p["args"]
        ):
            roots.add(pid)
    targets = set(roots)
    changed = True
    while changed:  # 대상의 자식(ffmpeg 등)도 함께 내린다
        changed = False
        for pid, p in procs.items():
            if (
                pid not in targets
                and p["ppid"] in targets
                and not any(x in p["args"] for x in RESTART_EXCLUDE)
            ):
                targets.add(pid)
                changed = True
    return {pid: (procs[pid]["exe"].name, procs[pid]["ticks"]) for pid in targets}


def alive(pid, ticks):
    try:
        stat = (Path("/proc") / str(pid) / "stat").read_text(encoding="ascii")
        rest = stat[stat.rfind(")") + 2 :].split()
        return rest[0] != "Z" and int(rest[19]) == ticks
    except (OSError, ValueError, IndexError):
        return False


def router_ready():
    device = ROOT / "config/device/device.json"
    port = json.loads(device.read_text())["router_gateway_port"] if device.is_file() else 14560
    address = "0100007F:" + format(int(port), "04X")
    rows = Path("/proc/net/udp").read_text().splitlines()[1:]
    return any(len(r.split()) > 1 and r.split()[1].upper() == address for r in rows)


def launch(script):
    import subprocess

    run = ROOT / "run/connection"
    run.mkdir(parents=True, exist_ok=True)
    with (run / (script + ".connection.log")).open("ab", buffering=0) as log:
        return subprocess.Popen(
            ["bash", str(ROOT / script)],
            cwd=ROOT,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )


def gateway_state():
    import urllib.request

    with urllib.request.urlopen("http://127.0.0.1:8768/state", timeout=1.5) as response:
        state = json.load(response)
    if state.get("app") != "argus-real-fc-gui":
        raise OSError("8768 belongs to another service")
    return state


def restart_worker():
    started = time.time()
    log = []

    def note(text, phase):
        log.append(time.strftime("%H:%M:%S ") + text)
        print(log[-1], flush=True)
        write_restart(phase=phase, started_unix=started, log=log[-40:], worker_pid=os.getpid())

    note("전체 재시작 시작 · 기체 상태 확인 없이 선점", "stopping")
    targets = restart_targets()
    note(
        "대상 %d개: %s"
        % (len(targets), ", ".join(sorted("%s(%d)" % (n, p) for p, (n, _) in targets.items()))),
        "stopping",
    )
    for pid in targets:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and any(alive(p, t) for p, (_, t) in targets.items()):
        time.sleep(0.1)
    left = {p: v for p, v in targets.items() if alive(p, v[1])}
    if left:
        note(
            "SIGTERM 3초 후 남은 %d개 SIGKILL: %s"
            % (len(left), ", ".join("%s(%d)" % (n, p) for p, (n, _) in left.items())),
            "stopping",
        )
        for pid in left:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and any(alive(p, t) for p, (_, t) in left.items()):
            time.sleep(0.1)
    still = [n for p, (n, t) in targets.items() if alive(p, t)]
    note("종료 완료" if not still else "종료 안 된 프로세스: " + ", ".join(still), "starting")
    # 2026-11-12 SIGKILL로 내린 DDS 참여자는 /dev/shm/fastrtps_* 파일을 남긴다. Fast DDS 도구로
    # 아무 프로세스도 쓰지 않는 파일만 지운다(사용 중인 파일은 건드리지 않음). 실패해도 재시작은 계속.
    import subprocess

    try:
        out = subprocess.run(
            ["fastdds", "shm", "clean"], capture_output=True, text=True, timeout=10
        )
        note(
            "DDS 공유메모리 정리 "
            + ("완료" if out.returncode == 0 else "실패(코드 %d) · 계속" % out.returncode),
            "starting",
        )
    except (OSError, subprocess.SubprocessError) as error:
        note("DDS 공유메모리 정리 생략(%s) · 계속" % error, "starting")
    # 게이트웨이(--router)는 라우터(UDP 14560)를 거쳐 FC에 붙는다. 라우터는 내리지 않고, 없으면 GUI 연결
    # 스크립트(connection.py)와 같은 방식으로 띄운다. FC USB가 빠져 있으면 라우터는 실패하고 계속 진행.
    if not router_ready():
        launch("run_router.sh")
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline and not router_ready():
            time.sleep(0.2)
        note(
            "라우터 " + ("준비됨" if router_ready() else "미준비(FC USB·UART 확인) · 계속"),
            "starting",
        )
    gateway = launch("run_pi_fc.sh")
    state = None
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        try:
            state = gateway_state()
            break
        except (OSError, ValueError):
            if gateway.poll() is not None:
                break
            time.sleep(0.3)
    if state is None:
        note("게이트웨이 기동 실패(run/connection/run_pi_fc.sh.connection.log 확인)", "failed")
        return
    note("게이트웨이 응답 확인", "starting")
    # 제어기·추적기·영상은 메인 GUI 접속과 FC 연결 뒤 게이트웨이가 띄운다. FC가 없으면 기다리지 않는다.
    deadline = time.monotonic() + 30
    names = set()
    fc = False
    while time.monotonic() < deadline:
        names = {p["exe"].name for p in user_processes().values()}
        if all(n in names for n in RESTART_WAIT):
            break
        try:
            fc = gateway_state().get("fresh") is True
        except (OSError, ValueError):
            fc = False
        if not fc and time.monotonic() > deadline - 20:
            break
        time.sleep(0.5)
    missing = [n for n in RESTART_WAIT if n not in names]
    if not missing:
        note("전체 재시작 완료 (%.1f초)" % (time.time() - started), "done")
    elif not fc:
        note(
            "게이트웨이 재기동 완료 · FC 미연결이라 제어기·추적기·영상은 FC 연결과 메인 GUI 접속 뒤 기동",
            "done",
        )
    else:
        note("게이트웨이는 기동 · 아직 없는 프로세스: " + ", ".join(missing), "partial")


def start_restart():
    import subprocess

    current = restart_status() or {}
    worker = current.get("worker_pid")
    if (
        current.get("phase") in ("stopping", "starting")
        and type(worker) is int
        and worker > 1
        and (Path("/proc") / str(worker)).exists()
        and time.time() - current.get("updated_unix", 0) < 60
    ):
        return dict(accepted=True, reason="전체 재시작 진행 중", restart=current)
    RESTART_LOG.parent.mkdir(parents=True, exist_ok=True)
    with RESTART_LOG.open("ab", buffering=0) as log:
        subprocess.Popen(
            [sys.executable, "-B", str(Path(__file__).resolve()), "restart-worker"],
            cwd=ROOT,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
    write_restart(
        phase="requested",
        started_unix=time.time(),
        log=[time.strftime("%H:%M:%S ") + "전체 재시작 요청 접수"],
    )
    return dict(
        accepted=True,
        reason="전체 재시작 요청 접수 · 게이트웨이 재연결까지 약 10~30초",
        restart=restart_status(),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("list", "stop", "restart", "restart-worker"))
    parser.add_argument("--pid", type=int)
    parser.add_argument("--start-ticks", type=int)
    a = parser.parse_args()
    if a.action == "restart-worker":
        try:
            restart_worker()
        except Exception as exc:
            write_restart(phase="failed", log=["재시작 작업 오류: " + str(exc)])
        return
    try:
        if a.action == "list":
            result = dict(accepted=True, processes=inventory(), restart=restart_status())
        elif a.action == "restart":
            result = start_restart()
        else:
            if a.pid is None or a.pid <= 1 or a.start_ticks is None or a.start_ticks <= 0:
                raise ValueError("프로세스 식별 오류")
            result = stop(a.pid, a.start_ticks)
    except Exception as exc:
        result = dict(accepted=False, reason=str(exc))
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
