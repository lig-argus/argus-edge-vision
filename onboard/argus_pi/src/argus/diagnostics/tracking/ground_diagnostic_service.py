# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Explicit, read-only IR ground diagnostic lifecycle; never starts or owns FC control."""

from pathlib import Path
import importlib.util
import json
import os
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
RUN = ROOT / "run" / "target_diagnostic"
BASE = ROOT / "diagnostics" / "target_manager"
RECORDER = ROOT / "src/argus/diagnostics/tracking/ground_recorder.py"


def active_pid():
    try:
        pid = int((RUN / "diagnostic.pid").read_text(encoding="ascii"))
        args = (Path("/proc") / str(pid) / "cmdline").read_bytes().split(b"\0")
        return pid if os.fsencode(str(RECORDER)) in args else None
    except (OSError, ValueError):
        return None


def status():
    try:
        folder = Path((RUN / "diagnostic-session.txt").read_text(encoding="utf8")).resolve()
        if not folder.is_relative_to(BASE.resolve()):
            raise ValueError("diagnostic output outside base")
        result = json.loads((folder / "status.json").read_text(encoding="utf8"))
    except (OSError, ValueError, json.JSONDecodeError):
        result = dict(reason="not_started")
    result.update(active=bool(active_pid()), available=True, flight_commands=False)
    return result


def request(action, state):
    import fcntl
    from tools.real_fc.runtime_settings import settings
    from tools.real_fc.follow_proxy import forward_vmy, validate_status

    RUN.mkdir(parents=True, exist_ok=True)
    with (RUN / "control.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if action == "status":
            return dict(accepted=True, **status())
        if action == "stop":
            pid = active_pid()
            if pid is not None:
                os.kill(pid, signal.SIGTERM)
            return dict(accepted=True, reason="지상 진단 기록 종료 요청", diagnostic=status())
        if action != "start":
            raise ValueError("unknown diagnostic action")
        owner = state.get("follow_b") or {}
        if not (
            state.get("fresh") is True
            and state.get("identity_ok") is True
            and state.get("armed") is False
            and state.get("ground") is True
            and not state.get("busy")
            and not state.get("owned_flight")
            and not state.get("external")
            and not owner.get("owner_active")
            and not owner.get("bench_active")
            and not (state.get("manual_yaw") or {}).get("active")
        ):
            raise ValueError(
                "진단 기록은 신원 확인된 지상·무장 해제·비행 제어 중단 상태에서만 가능합니다"
            )
        if active_pid():
            return dict(accepted=True, reason="진단 기록 이미 실행 중", diagnostic=status())
        config = settings()
        if state.get("expected_uid") != config["device"]["expected_fc_uid"]:
            raise ValueError("진단 대상 FC UID 불일치")
        current = validate_status(
            forward_vmy(
                dict(action="status", expected_uid=state["expected_uid"]), state["expected_uid"]
            ),
            state["expected_uid"],
        )
        if (
            current.get("active") is not False
            or current.get("bench_active") is not False
            or current.get("fc_epoch") != owner.get("fc_epoch")
        ):
            raise ValueError("진단 중 추종 비활성 및 FC epoch 확인 필요")
        mode = current.get("mode")
        if mode not in ("moving", "yaw"):
            raise ValueError("진단할 현재 추종 모드 미확인")
        binary_dir = Path(config["binary_dir"]).resolve()
        controller = (binary_dir / "argus_follow_controller").resolve()
        diagnostic = (binary_dir / "argus_target_diagnostic").resolve()
        if not diagnostic.is_file():
            raise ValueError("IR 진단 실행 파일이 없습니다. 해당 runtime을 빌드하세요")
        spec = importlib.util.spec_from_file_location(
            "argus_process_supervisor", ROOT / "src/argus/runtime/process_supervisor.py"
        )
        processes = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(processes)
        pid = processes.matching_process(controller)
        port = int(config["endpoints"]["operator"].rsplit(":", 1)[1])
        if pid is None or not processes.owns_listeners(pid, (port,)):
            raise ValueError("실행 중인 IR 추종기와 포트 소유권을 확인할 수 없습니다")
        args = (Path("/proc") / str(pid) / "cmdline").read_bytes().decode().split("\0")
        flag = "--moving-profile" if mode == "moving" else "--yaw-profile"
        try:
            profile = Path(args[args.index(flag) + 1]).resolve(strict=True)
        except (ValueError, IndexError, OSError) as exc:
            raise ValueError("실행 중인 추종 프로필을 확인할 수 없습니다") from exc
        if not profile.is_relative_to((ROOT / "run/config").resolve()):
            raise ValueError("진단 프로필이 IR runtime 범위 밖입니다")
        folder = BASE / (time.strftime("%Y%m%d_%H%M%S") + "_" + str(time.monotonic_ns()))
        BASE.mkdir(parents=True, exist_ok=True)
        command = [
            sys.executable,
            "-B",
            str(RECORDER),
            "--output",
            str(folder),
            "--profile",
            str(profile),
            "--binary",
            str(diagnostic),
            "--mode",
            mode,
            "--tracks",
            config["endpoints"]["tracks"],
            "--images",
            config["endpoints"]["vision"],
        ]
        with (RUN / "diagnostic.log").open("ab", buffering=0) as log:
            child = subprocess.Popen(
                command,
                cwd=ROOT,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=log,
                start_new_session=True,
            )
        (RUN / "diagnostic.pid").write_text(str(child.pid), encoding="ascii")
        (RUN / "diagnostic-session.txt").write_text(str(folder), encoding="utf8")
        return dict(
            accepted=True,
            reason="지상 진단 준비 · 대상만 선택하세요",
            output=str(folder),
            flight_commands=False,
        )
