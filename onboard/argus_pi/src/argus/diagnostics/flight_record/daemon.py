# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Passive FC lifecycle observer, raw evidence writer and idle archive supervisor."""

from __future__ import annotations
import argparse
import collections
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import msgpack
import zmq

from .storage import Session, archive, atomic_json, catalog, owned_directory, retain, sanitize


# 2026-11-25 batch 2: tracking/tracks, tracking/status 는 팀 규약 [이름][protobuf] (6375).
# 2026-11-25 batch 4: perception/observations 도 팀 vision 6374 [이름][protobuf] (argus-onboard 가 bind).
# 2026-11-25 batch 6a: fc/state, fc/attitude, fc/mode_event 도 팀 6370 [이름][protobuf].
# 2026-11-25 batch 6b: control/setpoint (6372, control/ 접두 구독) 도 팀 [이름][protobuf]. 기록기는
# follow/status 를 구독하지 않는다 (예전과 같다). 이제 구독하는 토픽은 모두 protobuf 다.
PROTOBUF_TOPICS = (
    "tracking/tracks",
    "tracking/status",
    "perception/observations",
    "fc/state",
    "fc/attitude",
    "fc/mode_event",
    "control/setpoint",
)


def load_pb_json(root):
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "argus_pb_json", root / "src/argus/transport/pb_json.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_config(root):
    c = json.loads((root / "config/flight_recording.json").read_text())
    if (
        c.get("schema") != "argus.flight.recording.config.v1"
        or type(c.get("enabled")) is not bool
        or type(c.get("retain_sessions")) is not int
        or not 1 <= c["retain_sessions"] <= 10
        or not 0 <= c["prebuffer_seconds"] <= 5
        or not 0 < c["prebuffer_bytes"] <= 128 * 1024**2
        or c["minimum_free_bytes"] < 0
        or not 1 <= c["compression_level"] <= 3
        or not 0.2 <= c["landed_disarmed_confirm_seconds"] <= 5
        or not 0 <= c["postflight_seconds"] <= 15
    ):
        raise ValueError("invalid flight recording config")
    return c


def snapshot(root, session):
    files = {}
    candidates = list((root / "config").rglob("*"))
    for n in [
        "src/argus/perception/detection/python_vision_worker.py",
        "src/argus/perception/detection/yolox_detector.py",
        "src/argus/perception/preprocessing/preprocessor.py",
        "src/argus/perception/preprocessing/prep_fast.py",
        "src/argus/perception/geometry/pi_ground_projector.cpp",
        "src/argus/perception/geometry/frame_transform.cpp",
        "src/argus/runtime/follow/pi_tracking_main.cpp",
        "src/argus/control/guidance/predictive_follower.cpp",
        "src/argus/control/guidance/flight_guidance_controller.cpp",
        "src/argus/control/target/target_manager.cpp",
        "src/argus/tracking/motion/short_motion_estimator.cpp",
        "src/argus/control/target/include/cv_motion_cache.hpp",
    ]:
        candidates.append(root / n)
    for p in candidates:
        if not p.is_file() or p.is_symlink() or "__pycache__" in p.parts:
            continue
        if p.stat().st_size > 4 * 1024**2:
            continue
        rel = p.relative_to(root)
        data = p.read_bytes()
        files[str(rel)] = hashlib.sha256(data).hexdigest()
        dest = session.folder / "configuration" / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
    session.meta["configuration_sha256"] = files
    session.meta["limits"] = dict(
        original_exposure_timing="unverified unless source explicitly certifies",
        FC_ulog="stored on FC separately; not an automatic ULog download",
        observation="raw SDK pixels at inference input; source frames skipped before inference are not recoverable",
    )
    session.save_meta()


class Lifecycle:
    def __init__(self, confirm=1, tail=5):
        self.confirm, self.tail = confirm, tail
        self.ground_since = None
        self.finish_at = None

    def step(self, state, fresh, now):
        if not fresh:
            self.ground_since = None
            self.finish_at = None
            return False
        grounded = (
            state.get("ground_known") is True
            and state.get("landed") is True
            and state.get("armed") is False
            and state.get("in_air") is False
        )
        if not grounded:
            self.ground_since = None
            self.finish_at = None
            return False
        if self.ground_since is None:
            self.ground_since = now
        if now - self.ground_since >= self.confirm and self.finish_at is None:
            self.finish_at = now + self.tail
        return self.finish_at is not None and now >= self.finish_at


class Tail:
    def __init__(self, path):
        self.path = Path(path)
        self.file = None
        self.inode = None
        self.offset = 0

    def read(self):
        output = []
        try:
            stat = self.path.stat()
            ident = (stat.st_dev, stat.st_ino)
            if self.file is None:
                self.file = self.path.open("rb")
                self.file.seek(0, 2)
                self.offset = self.file.tell()
                self.inode = ident
                return []
            for _ in range(512):
                at = self.file.tell()
                line = self.file.readline(2 * 1024**2)
                if not line:
                    break
                if not line.endswith(b"\n"):
                    if len(line) < 2 * 1024**2:
                        self.file.seek(at)
                        break
                    # An oversized malformed line must not wedge this tail forever.
                    skipped = len(line)
                    while line and not line.endswith(b"\n"):
                        line = self.file.readline(2 * 1024**2)
                        skipped += len(line)
                    output.append(
                        dict(
                            file=str(self.path.name),
                            inode=list(self.inode),
                            byte_offset=at,
                            line=json.dumps(
                                {
                                    "schema": "argus.flight.tail.loss.v1",
                                    "reason": "oversized diagnostic line",
                                    "bytes": skipped,
                                }
                            ),
                        )
                    )
                    continue
                output.append(
                    dict(
                        file=str(self.path.name),
                        inode=list(self.inode),
                        byte_offset=at,
                        line=line.decode("utf8", errors="replace").rstrip("\r\n"),
                    )
                )
            if ident != self.inode:
                # Keep draining the old inode before opening the new rotation at byte zero.
                here = self.file.tell()
                more = self.file.read(1)
                self.file.seek(here)
                if not more:
                    rotations = []
                    for n in range(1, 4):
                        p = Path(str(self.path) + "." + str(n))
                        try:
                            s = p.stat()
                            rotations.append((n, p, (s.st_dev, s.st_ino)))
                        except FileNotFoundError:
                            pass
                    old = next((n for n, p, i in rotations if i == self.inode), None)
                    next_path = self.path
                    if old is not None and old > 1:
                        next_path = next((p for n, p, i in rotations if n == old - 1), self.path)
                    elif old is None and rotations:
                        next_path = max(rotations, key=lambda x: x[0])[1]
                        output.append(
                            dict(
                                file=str(self.path.name),
                                inode=list(self.inode),
                                byte_offset=here,
                                line=json.dumps(
                                    {
                                        "schema": "argus.flight.tail.loss.v1",
                                        "reason": "old diagnostic inode no longer retained; rotation gap possible",
                                    }
                                ),
                            )
                        )
                    self.file.close()
                    self.file = next_path.open("rb")
                    s = os.fstat(self.file.fileno())
                    self.inode = (s.st_dev, s.st_ino)
            elif stat.st_size < self.file.tell():
                self.file.seek(0)
        except FileNotFoundError:
            pass
        return output

    def close(self):
        if self.file:
            self.file.close()


class Recorder:
    def __init__(self, root, config, uid):
        self.root, self.config, self.uid = root, config, uid
        self.folder = root / "recordings/flights"
        self.folder.mkdir(parents=True, exist_ok=True)
        self.status_path = root / "run/flight-recorder/status.json"
        self.current = None
        self.binding = None
        self.state = {}
        self.state_received = 0
        self.pre = collections.deque()
        self.pre_bytes = 0
        self.errors = []
        self.last_error = None
        self.life = Lifecycle(
            config["landed_disarmed_confirm_seconds"], config["postflight_seconds"]
        )
        self.archive_process = None
        self.archive_folder = None
        self.archive_log = None
        self.next_archive = 0
        self.started = time.time_ns()
        self.total = collections.Counter()
        self.stopping = False
        self.last_raw_seq = {}
        self.low_space = shutil.disk_usage(self.folder).free < config["minimum_free_bytes"]
        self.saw_landed_disarmed = False
        self.session_retry_at = 0
        self.source_state_time_ns = 0

    def fresh(self, now):
        return (
            self.state.get("schema") == "argus.fc.state.v1"
            and self.state.get("uid") == self.uid
            and self.state.get("connected") is True
            and self.state.get("clock_domain") == "host_monotonic"
            and 0 <= now - self.state_received <= 0.5
            and type(self.state.get("fc_epoch")) is int
            and type(self.state.get("received_monotonic_ns")) is int
            and 0 <= now * 1e9 - self.state["received_monotonic_ns"] <= 500_000_000
            and bool(self.state.get("onboard_session"))
        )

    def ground_safe(self, now):
        return (
            self.fresh(now)
            and self.state.get("armed") is False
            and self.state.get("ground_known") is True
            and self.state.get("landed") is True
            and self.state.get("in_air") is False
        )

    def record_error(self, reason):
        self.last_error = str(reason)
        if self.current and str(reason) not in self.current.meta["errors"]:
            self.current.meta["errors"].append(str(reason))
        if not self.errors or self.errors[-1] != str(reason):
            self.errors.append(str(reason))
            self.errors = self.errors[-20:]

    def fc_state(self, packet, received):
        source_stamp = packet.get("received_monotonic_ns")
        if type(source_stamp) is not int or not 0 <= received * 1e9 - source_stamp <= 500_000_000:
            self.record_error("FC state source stale/future/missing; not used for flight boundary")
            return
        self.state = packet
        self.state_received = received
        binding = (packet.get("onboard_session"), packet.get("fc_epoch"))
        if self.current and self.fresh(received) and binding != self.binding:
            self.finish_session("FC epoch/session changed; landing not proven", True)
        if self.current and self.ground_safe(received):
            self.saw_landed_disarmed = True
        if (
            self.current
            and self.fresh(received)
            and packet.get("armed") is True
            and self.saw_landed_disarmed
        ):
            confirmed = self.life.finish_at is not None
            self.current.meta["postflight_tail_cut_by_rearm"] = True
            self.finish_session(
                "actual landing/disarm observed; tail ended by next arm"
                if confirmed
                else "rearmed before landing/disarm confirmation completed",
                not confirmed,
            )
        armed = self.fresh(received) and packet.get("armed") is True
        airborne = (
            armed
            and packet.get("in_air") is True
            and packet.get("ground_known") is True
            and packet.get("landed") is False
        )
        if armed and self.current is None and received >= self.session_retry_at:
            self.cancel_archive()
            try:
                self.current = Session(
                    self.folder, self.uid, binding, "actual_armed", self.config["retain_sessions"]
                )
                self.binding = binding
                self.saw_landed_disarmed = False
                self.life = Lifecycle(
                    self.config["landed_disarmed_confirm_seconds"],
                    self.config["postflight_seconds"],
                )
                self.low_space = (
                    shutil.disk_usage(self.folder).free < self.config["minimum_free_bytes"]
                )
                try:
                    snapshot(self.root, self.current)
                except Exception as exc:
                    self.record_error("configuration snapshot incomplete: " + str(exc))
                for _, stream, data, blobs, _ in self.pre:
                    self.write(stream, data, blobs)
                self.pre.clear()
                self.pre_bytes = 0
            except Exception as exc:
                self.record_error("session start: " + str(exc))
                self.session_retry_at = received + 2
                if self.current:
                    self.finish_session("session initialization failed", True)

        if airborne and self.current:
            self.current.meta["flight_observed"] = True

    def finish_session(self, reason, incomplete=False):
        current = self.current
        self.current = None
        self.binding = None
        self.saw_landed_disarmed = False
        if current is None:
            return
        try:
            current.meta["counts"]["recorder_totals"] = dict(self.total)
            current.close(reason, incomplete)
        except Exception as exc:
            self.record_error(
                "session close incomplete; raw retained: " + str(current.folder) + ": " + str(exc)
            )

    def write(self, stream, data, blobs=()):
        if not self.current:
            return
        try:
            if stream == "pixels":
                if self.low_space:
                    self.total["raw_dropped_disk_pressure"] += 1
                    self.record_error(
                        "raw recording paused: disk free space below configured reserve"
                    )
                    return
                source = (data.get("source"), data.get("header", {}).get("session_id"))
                seq = data.get("header", {}).get("frame_id")
                old = self.last_raw_seq.get(source)
                if type(seq) is int:
                    if old is not None and seq > old + 1:
                        self.total["observed_source_frame_gaps"] += seq - old - 1
                    self.last_raw_seq[source] = seq
                if self.current.pixels(data, *blobs):
                    self.total["raw_frames_written"] += 1
                else:
                    self.total["raw_frames_ignored_after_source_cutoff"] += 1
            else:
                self.current.line(stream + ".jsonl", data)
        except Exception as exc:
            self.record_error("write: " + str(exc))
            self.low_space = True

    def feed(self, stream, data, blobs=(), now=None):
        now = time.monotonic() if now is None else now
        data = sanitize(data)
        if self.current:
            self.write(stream, data, blobs)
        else:
            size = sum(len(x) for x in blobs) + len(
                json.dumps(data, ensure_ascii=False, default=str)
            )
            self.pre.append((now, stream, data, blobs, size))
            self.pre_bytes += size
            while self.pre and (
                now - self.pre[0][0] > self.config["prebuffer_seconds"]
                or self.pre_bytes > self.config["prebuffer_bytes"]
            ):
                self.pre_bytes -= self.pre.popleft()[4]

    def cancel_archive(self):
        if self.archive_process and self.archive_process.poll() is None:
            self.archive_process.terminate()
            try:
                self.archive_process.wait(2)
            except subprocess.TimeoutExpired:
                self.archive_process.kill()
                self.archive_process.wait(2)
            self.record_error(
                "archive paused for FC armed/airborne/unknown; raw retained for retry"
            )
        if self.archive_log:
            self.archive_log.close()
        self.archive_process = None
        self.archive_folder = None
        self.archive_log = None

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        if self.current:
            finished = self.life.step(self.state, self.fresh(now), now)
            if self.life.finish_at is not None:
                self.current.freeze_sources(int(now * 1e9))
            if finished:
                self.finish_session("actual_landed_disarmed_confirmed_with_tail")
            else:
                self.current.flush()
        if self.archive_process:
            if not self.ground_safe(now):
                self.cancel_archive()
            elif self.archive_process.poll() is not None:
                rc = self.archive_process.returncode
                if rc:
                    self.record_error(
                        "ZIP verification failed; source retained: " + str(self.archive_folder)
                    )
                self.archive_log.close()
                self.archive_process = None
                self.archive_log = None
                self.archive_folder = None
                self.next_archive = now + 5
        self.low_space = shutil.disk_usage(self.folder).free < self.config["minimum_free_bytes"]
        if (
            not self.current
            and not self.archive_process
            and self.ground_safe(now)
            and now >= self.next_archive
        ):
            for path in sorted(self.folder.glob("flight-*.open")):
                try:
                    try:
                        metadata = owned_directory(self.folder, path, self.uid)
                    except (OSError, ValueError, KeyError):
                        # The worker validates the verified archive and hashes every
                        # remaining source file before retrying interrupted raw cleanup.
                        sidecar = self.folder / (path.name[:-5] + ".verified.json")
                        if sidecar.is_symlink() or not sidecar.is_file():
                            raise
                        saved = json.loads(sidecar.read_text())
                        if (
                            saved.get("uid") != self.uid
                            or saved.get("verified") is not True
                            or saved.get("session_id") != path.name[:-5]
                        ):
                            raise
                        metadata = {"closed": True}
                    if not metadata.get("closed"):
                        continue
                    self.archive_folder = path
                    self.archive_log = (self.root / "run/flight-recorder/archive.log").open("ab")
                    env = dict(
                        os.environ,
                        PYTHONPATH=str(self.root / "src")
                        + os.pathsep
                        + os.environ.get("PYTHONPATH", ""),
                    )
                    command = [
                        sys.executable,
                        "-B",
                        "-m",
                        "argus.diagnostics.flight_record.archive_worker",
                        "--root",
                        str(self.folder),
                        "--session",
                        str(path),
                        "--uid",
                        self.uid,
                        "--level",
                        str(self.config["compression_level"]),
                        "--retain",
                        str(self.config["retain_sessions"]),
                    ]
                    self.archive_process = subprocess.Popen(
                        command, stdout=self.archive_log, stderr=self.archive_log, env=env
                    )
                    break
                except Exception as exc:
                    self.record_error("archive start: " + str(exc))
                    self.next_archive = now + 5
                    if self.archive_log:
                        self.archive_log.close()
                    self.archive_log = None
                    self.archive_folder = None
        atomic_json(
            self.status_path,
            dict(
                schema="argus.flight.recorder.status.v1",
                pid=os.getpid(),
                received_monotonic_ns=time.monotonic_ns(),
                enabled=True,
                active_session=None if self.current is None else str(self.current.folder),
                waiting_for=("verified fresh FC" if not self.fresh(now) else "ARM")
                if self.current is None
                else "actual landing and disarm",
                archive_session=None if self.archive_folder is None else str(self.archive_folder),
                logical_sessions=len(catalog(self.folder, self.uid)),
                retain_sessions=self.config["retain_sessions"],
                minimum_free_bytes=self.config["minimum_free_bytes"],
                low_disk_space=self.low_space,
                counts=dict(self.total),
                last_error=self.last_error,
                recent_errors=self.errors,
                active_evidence=None
                if self.current is None
                else dict(
                    raw_frame_keys=len(self.current.raw_keys),
                    cmc_frame_keys=len(self.current.cmc_keys),
                    follow_decision_records=self.current.follow_decision_records,
                ),
                FC_commands=False,
                exposure_alignment_verified=False,
            ),
        )


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--check", action="store_true")
    a = p.parse_args(argv)
    root = a.root.resolve()
    cfg = load_config(root)
    sys.path.insert(0, str(root))
    from deployment_config import load

    settings = load(root)
    uid = settings["device"]["expected_fc_uid"]
    if a.check:
        print(
            json.dumps(
                dict(config=cfg, root=str(root), uid=uid, opens_devices=False, FC_commands=False)
            )
        )
        return 0
    if not cfg["enabled"]:
        return 0
    run = root / "run/flight-recorder"
    run.mkdir(parents=True, exist_ok=True)
    lock = (run / "recorder.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return 0
    try:
        os.nice(5)
    except OSError:
        pass
    rec = Recorder(root, cfg, uid)
    for folder in rec.folder.glob("flight-*.open"):
        try:
            m = owned_directory(rec.folder, folder, uid)
            if not m.get("closed"):
                m.update(
                    closed=True,
                    incomplete=True,
                    ended_reason="recorder restart/power interruption; landing not observed",
                    closed_unix_ns=time.time_ns(),
                )
                atomic_json(folder / "session.json", m)
        except Exception as exc:
            rec.record_error("recovery: " + str(exc))
    context = zmq.Context()
    pull = context.socket(zmq.PULL)
    pull.setsockopt(zmq.LINGER, 0)
    pull.setsockopt(zmq.RCVHWM, 8)
    # 2026-11-25 batch 6c: ZMQ_MAXMSGSIZE 없음 (libzmq 4.3.5 는 넘는 프레임을 보낸 연결을 끊고 다시 맺지 않는다).
    # 상한 (tap 16 MiB, 버스 본문 2 MiB) 은 받은 뒤 보고 그 메시지만 버리고 센다 (bus_oversized, 오류 기록)
    endpoint = "ipc://" + str(run / "pixels.sock")
    pull.bind(endpoint)
    sub = context.socket(zmq.SUB)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.RCVHWM, 512)
    for name in ["fc_state", "tracks", "vision", "control"]:
        sub.connect(settings["endpoints"][name])
    for topic in [b"fc/", b"tracking/", b"perception/observations", b"control/"]:
        sub.setsockopt(zmq.SUBSCRIBE, topic)
    poll = zmq.Poller()
    poll.register(sub, zmq.POLLIN)
    poll.register(pull, zmq.POLLIN)
    pb_json = load_pb_json(root)
    tails = {}
    last_tail = last_tick = 0

    def stop(*_):
        rec.stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while not rec.stopping:
            ready = dict(poll.poll(20))
            now = time.monotonic()
            if sub in ready:
                for _ in range(256):
                    try:
                        parts = sub.recv_multipart(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    if len(parts) != 2:
                        continue
                    if len(parts[1]) > 2 * 1024**2:
                        rec.total["bus_oversized"] += 1
                        rec.record_error("bus message: body over 2 MiB")
                        continue
                    try:
                        topic = parts[0].decode()
                        # 기록기는 받은 값을 그대로 남긴다 (pi_schema 검사는 판단하는 소비자가 한다).
                        # 2026-11-25 batch 4 리뷰: Pi 소비자가 받지 않는 본문이면 pi_rejected 에 까닭을 남긴다
                        value, rejected = (
                            pb_json.decode_for_record(topic, parts[1])
                            if topic in PROTOBUF_TOPICS
                            else (msgpack.unpackb(parts[1], raw=False), None)
                        )
                        if not isinstance(value, dict):
                            continue
                        received = time.monotonic_ns()
                        if topic == "fc/state":
                            rec.fc_state(value, now)
                        entry = dict(
                            topic=topic, recorder_received_monotonic_ns=received, packet=value
                        )
                        if rejected:
                            entry["pi_rejected"] = rejected
                        rec.feed("bus", entry, now=now)
                    except Exception as exc:
                        rec.record_error("bus message: " + str(exc))
            if pull in ready:
                for _ in range(8):
                    try:
                        parts = pull.recv_multipart(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    try:
                        if len(parts) != 5 or parts[0] != b"pixels":
                            raise ValueError("tap multipart shape")
                        if sum(map(len, parts)) > 16 * 1024**2:
                            raise ValueError("tap message over 16 MiB")
                        m = msgpack.unpackb(parts[1], raw=False)
                        h = m["header"]
                        if (
                            m.get("schema") != "argus.flight.pixels.v1"
                            or h.get("pixel_format") not in ("RAW14_LE16", "BGR8")
                            or h.get("width") != 640
                            or h.get("height") != 480
                            or len(parts[2]) != h["stride"] * h["height"]
                        ):
                            raise ValueError("source layout mismatch")
                        if not (
                            type(h.get("frame_id")) is int
                            and type(h.get("received_monotonic_ns")) is int
                        ):
                            raise ValueError("frame key invalid")
                        rec.total["tap_frames_received"] += 1
                        rec.total["tap_reported_dropped_total"] = max(
                            rec.total["tap_reported_dropped_total"], m.get("tap_dropped_total", 0)
                        )
                        rec.feed("pixels", m, tuple(parts[2:]), now)
                    except Exception as exc:
                        rec.record_error("pixel message: " + str(exc))
            if now - last_tail >= 0.1:
                last_tail = now
                paths = [
                    root / "control.csv",
                    root / "control.csv.target.csv",
                    root / "control.csv.decision.jsonl",
                    run / "cmc-stream.jsonl",
                    root / "run/main-gcs-bridge.jsonl",
                    root / "run/preview/producer.log",
                    root / "run/follow_b/runtime.log",
                    root / "run/follow_b/tracker.log",
                ]
                paths += list((root / "run/follow_b").glob("cv-owner-*.jsonl"))
                for path in paths:
                    tail = tails.setdefault(str(path), Tail(path))
                    for line in tail.read():
                        content = line.pop("line")
                        try:
                            line["record"] = json.loads(content)
                        except ValueError:
                            line["text"] = content
                        line["recorder_received_monotonic_ns"] = time.monotonic_ns()
                        rec.feed(
                            "cmc" if path.name == "cmc-stream.jsonl" else "follow_diagnostics",
                            line,
                            now=now,
                        )
            if now - last_tick >= 0.2:
                last_tick = now
                try:
                    rec.tick(now)
                except Exception as exc:
                    rec.record_error("recorder supervision: " + str(exc))
    finally:
        rec.cancel_archive()
        if rec.current:
            rec.finish_session("recorder stopped before completed landing boundary", True)
        for tail in tails.values():
            tail.close()
        sub.close(0)
        pull.close(0)
        context.term()
        atomic_json(
            run / "status.json",
            dict(enabled=False, pid=os.getpid(), last_error=rec.last_error, FC_commands=False),
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
