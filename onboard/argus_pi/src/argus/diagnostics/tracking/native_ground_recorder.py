# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Read-only native-ICD ground shadow recorder. No FC commands or output sockets."""

from pathlib import Path
import atexit
import hashlib
import json
import os
import select
import subprocess
import sys
import time
import uuid

import msgpack
import zmq

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from deployment_config import load


def load_pb_json():
    """팀 [이름][protobuf] 해석기 (src/argus/transport/pb_json.py).

    2026-11-25 batch 2: tracking/tracks. 2026-11-25 batch 6a: fc/state (6370).
    """
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "argus_pb_json", ROOT / "src/argus/transport/pb_json.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def emit(**values):
    print(
        json.dumps(
            dict(schema="argus.native_ground_status.v1", **values),
            ensure_ascii=False,
            allow_nan=False,
        ),
        flush=True,
    )


def valid_control(control, now, received, uid):
    target = control.get("target") or {}
    return (
        now - received <= 0.6
        and control.get("safe") is True
        and control.get("expected_uid") == uid
        and (
            target.get("sensor") in ("1", 1)
            or (
                isinstance(target.get("sensor"), str)
                and target["sensor"].startswith(("ir_sdk:", "ir_v4l2:"))
            )
        )
        and type(target.get("generation")) is int
        and 0 < target["generation"] < 16777216
        and type(target.get("track_id")) is int
        and 0 < target["track_id"] < 16777216
        and type(target.get("class_id")) is int
        and 0 <= target["class_id"] < 16777216
    )


def valid_fc(state, now_ns, uid):
    stamp = state.get("received_monotonic_ns")
    return (
        state.get("schema") == "argus.fc.state.v1"
        and state.get("clock_domain") == "host_monotonic"
        and state.get("uid") == uid
        and state.get("connected") is True
        and state.get("armed") is False
        and state.get("in_air") is False
        and state.get("ground_known") is True
        and state.get("landed") is True
        and state.get("offboard") is False
        and state.get("rc_override") is False
        and type(stamp) is int
        and 0 <= now_ns - stamp <= 500_000_000
        and type(state.get("fc_epoch")) is int
        and state["fc_epoch"] >= 0
        and type(state.get("onboard_session")) is str
        and bool(state["onboard_session"])
    )


def main():
    settings = load(ROOT)
    uid = settings["device"]["expected_fc_uid"]
    binary = Path(settings["binary_dir"]) / "argus_target_diagnostic"
    if not binary.is_file():
        raise RuntimeError("diagnostic_binary_missing")
    folder = (
        ROOT
        / "diagnostics/target_manager"
        / (time.strftime("%Y%m%d_%H%M%S") + "_native_" + uuid.uuid4().hex[:12])
    )
    folder.mkdir(parents=True, exist_ok=False)
    config = ROOT / "config"
    manifest = {
        str(p.relative_to(config)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in config.rglob("*")
        if p.is_file()
    }
    source_profile = Path(settings["paths"]["moving"])
    values = {
        "expected_fc_uid": uid,
        "expected_fc_system_id": settings["device"]["expected_fc_system_id"],
        "expected_fc_component_id": settings["device"]["expected_fc_component_id"],
        "tracks_sub_endpoint": settings["endpoints"]["tracks"],
        "tracks_pub_endpoint": settings["endpoints"]["tracks"],
    }
    lines = [
        line
        for line in source_profile.read_text(encoding="utf8").splitlines()
        if line.partition("=")[0] not in values
    ]
    profile = folder / "profile.config"
    profile.write_text(
        "\n".join(lines + [str(k) + "=" + str(v) for k, v in values.items()]) + "\n",
        encoding="utf8",
    )
    (folder / "manifest.json").write_text(
        json.dumps(
            dict(
                shadow_only=True,
                flight_commands=False,
                profile_sha256=hashlib.sha256(profile.read_bytes()).hexdigest(),
                config_sha256=manifest,
            ),
            indent=2,
        ),
        encoding="utf8",
    )
    stderr = (folder / "native.stderr.log").open("w")
    child = subprocess.Popen(
        [str(binary), str(profile)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=stderr,
        text=True,
        bufsize=1,
    )

    def reap_owned_shadow():
        if child.poll() is None:
            try:
                child.stdin.close()
            except OSError:
                pass
            try:
                child.terminate()
                child.wait(timeout=2)
            except (OSError, subprocess.TimeoutExpired):
                pass

    atexit.register(reap_owned_shadow)
    pb_json = load_pb_json()
    context = zmq.Context()
    sockets = []
    poll = zmq.Poller()
    for endpoint, topic in [
        (settings["endpoints"]["fc_state"], b"fc/state"),
        (settings["endpoints"]["tracks"], b"tracking/tracks"),
        (settings["endpoints"]["vision"], b"vision/overlay/jpeg"),
    ]:
        sock = context.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVHWM, 32)
        sock.connect(endpoint)
        sock.setsockopt(zmq.SUBSCRIBE, topic)
        poll.register(sock, zmq.POLLIN)
        sockets.append(sock)
    fc_socket, tracks, images = sockets
    input_fd = sys.stdin.fileno()
    os.set_blocking(input_fd, False)
    poll.register(input_fd, zmq.POLLIN)
    frames = (folder / "frames.jsonl").open("w", encoding="utf8")
    events = (folder / "events.jsonl").open("w", encoding="utf8")
    pending, pictures, control, fc = {}, {}, {}, {}
    buffer = b""
    last_control = 0.0
    last_emit = 0.0
    started = time.monotonic()
    count = paired = size = 0
    last_safe = False
    shadow_identity = None
    reason = "waiting_for_ground_fc_and_selection"

    def save(key, row, picture=None):
        nonlocal count, paired, size
        count += 1
        row["image"] = None
        row["image_missing_reason"] = "matching_frame_not_received"
        if picture:
            meta, jpeg = picture
            name = f"{count:06d}_{key[1]}.jpg"
            (folder / name).write_bytes(jpeg)
            size += len(jpeg)
            paired += 1
            row.update(
                image=name,
                image_sha256=hashlib.sha256(jpeg).hexdigest(),
                image_metadata=meta,
                image_missing_reason=None,
            )
        text = json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n"
        frames.write(text)
        frames.flush()
        size += len(text.encode("utf8"))

    def reset_shadow():
        child.stdin.write(json.dumps({"safety": {}}) + "\n")
        child.stdin.flush()
        if not select.select([child.stdout], [], [], 1)[0]:
            raise RuntimeError("shadow_reset_timeout")
        child.stdout.readline()

    try:
        while time.monotonic() - started < 600 and size < 500 * 1024 * 1024:
            now = time.monotonic()
            if now - (last_control or started) > 5:
                reason = "maintenance_connection_lost"
                break
            if os.statvfs(folder).f_bavail * os.statvfs(folder).f_frsize < 256 * 1024 * 1024:
                raise RuntimeError("free_space_limit")
            ready = dict(poll.poll(20))
            if input_fd in ready:
                part = os.read(input_fd, 8192)
                if not part:
                    reason = "maintenance_connection_closed"
                    break
                buffer += part
                if len(buffer) > 65536:
                    raise RuntimeError("control_input_too_large")
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    request = json.loads(line)
                    if request.get("action") == "stop":
                        reason = "operator_stopped"
                        return
                    if request.get("action") != "observe":
                        raise RuntimeError("unsupported_control_action")
                    control = request
                    last_control = time.monotonic()
            if fc_socket in ready:
                parts = fc_socket.recv_multipart()
                # 2026-11-25 batch 6a: 팀 6370 protobuf. 이름은 정확히 fc/state 만. 해석 실패는 예전
                # msgpack 해석 실패처럼 기록을 멈춘다 (ValueError)
                if len(parts) == 2 and parts[0] == b"fc/state" and len(parts[1]) < 1024 * 1024:
                    candidate = pb_json.decode("fc/state", parts[1])[0]
                    if isinstance(candidate, dict):
                        fc = candidate
            safe = valid_control(control, time.monotonic(), last_control, uid) and valid_fc(
                fc, time.monotonic_ns(), uid
            )
            if last_safe and not safe:
                for key, (_, row) in list(pending.items()):
                    save(key, row, pictures.get(key))
                pending.clear()
                pictures.clear()
                reset_shadow()
                shadow_identity = None
            if safe != last_safe:
                events.write(json.dumps(dict(monotonic_ns=time.monotonic_ns(), safe=safe)) + "\n")
                events.flush()
                last_safe = safe
            if images in ready:
                parts = images.recv_multipart()
                if (
                    safe
                    and len(parts) == 3
                    and parts[0] == b"vision/overlay/jpeg"
                    and len(parts[2]) < 2 * 1024 * 1024
                ):
                    # 2026-11-25 batch 4: 팀 vision 6374 protobuf (수와 1, 2, 4 ... 번째 로그는 pb_json)
                    try:
                        meta = pb_json.decode_pi_vision("vision/overlay/jpeg", parts[1])[0]
                    except ValueError:
                        meta = None
                    if meta is not None:
                        key = (meta["source"], meta["frame_id"], meta["received_monotonic_ns"])
                        pictures[key] = (meta, parts[2])
            if tracks in ready:
                parts = tracks.recv_multipart()
                raw = None
                if (
                    safe
                    and len(parts) == 2
                    and parts[0] == b"tracking/tracks"
                    and len(parts[1]) < 1024 * 1024
                ):
                    try:
                        raw = pb_json.decode_pi_tracks(parts[1])
                    except ValueError as exc:
                        reason = "tracks_decode_failed: " + str(exc)
                if raw is not None:
                    target = control["target"]
                    valid_source = (
                        isinstance(raw.get("source"), str)
                        and raw["source"].startswith(("ir_sdk:", "ir_v4l2:"))
                        and (
                            target.get("sensor") in ("1", 1)
                            or target.get("sensor") == raw["source"]
                        )
                    )
                    real_target = any(
                        t.get("track_id") == target["track_id"]
                        and t.get("class_id") == target["class_id"]
                        for t in raw.get("tracks", [])
                        if isinstance(t, dict)
                    )
                    identity = (
                        target["generation"],
                        target["track_id"],
                        target["class_id"],
                        fc["onboard_session"],
                    )
                    if (
                        not valid_source
                        or raw.get("generation") != target["generation"]
                        or raw.get("fc_epoch") != fc.get("fc_epoch")
                        or (shadow_identity != identity and not real_target)
                    ):
                        reason = "waiting_for_matching_ir_target_or_fc_epoch"
                    else:
                        chosen = {
                            "generation": target["generation"],
                            "track_id": target["track_id"],
                            "class_id": target["class_id"],
                            "onboard_session": fc["onboard_session"],
                        }
                        safety = {
                            "fresh": True,
                            "identity_ok": True,
                            "expected_uid": uid,
                            "armed": False,
                            "ground": True,
                            "busy": False,
                            "follow_active": False,
                        }
                        req = dict(
                            safety=safety,
                            selection=chosen,
                            tracks=raw,
                            received_ns=time.monotonic_ns(),
                        )
                        child.stdin.write(json.dumps(req, allow_nan=False) + "\n")
                        child.stdin.flush()
                        if not select.select([child.stdout], [], [], 1)[0]:
                            raise RuntimeError("shadow_response_timeout")
                        result = json.loads(child.stdout.readline())
                        reason = result.get("error") or result.get("reason", "recording")
                        if result.get("initialized") is True:
                            shadow_identity = identity
                        key = (raw["source"], raw["frame_id"], raw["sample_monotonic_ns"])
                        pending[key] = (time.monotonic(), dict(input=req, diagnostic=result))
            for key, (at, row) in list(pending.items()):
                if key in pictures:
                    save(key, row, pictures.pop(key))
                    del pending[key]
                elif time.monotonic() - at > 1:
                    save(key, row)
                    del pending[key]
            while len(pictures) > 64:
                del pictures[next(iter(pictures))]
            while len(pending) > 32:
                key = next(iter(pending))
                save(key, pending.pop(key)[1])
            if time.monotonic() - last_emit >= 1:
                last_emit = time.monotonic()
                emit(
                    active=True,
                    records=count,
                    paired_images=paired,
                    output=str(folder),
                    reason=reason if safe else "paused_ground_identity_or_maintenance_state",
                    flight_commands=False,
                )
        if reason == "waiting_for_ground_fc_and_selection":
            reason = "recording_limit"
    except Exception as exc:
        reason = "stopped: " + str(exc)
    finally:
        for key, (_, row) in pending.items():
            save(key, row, pictures.get(key))
        frames.close()
        events.close()
        for sock in sockets:
            sock.close()
        context.term()
        child.stdin.close()
        try:
            child.wait(timeout=2)
        except subprocess.TimeoutExpired:
            child.terminate()
            child.wait(timeout=2)
        stderr.close()
        emit(
            active=False,
            records=count,
            paired_images=paired,
            output=str(folder),
            reason=reason,
            flight_commands=False,
        )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        emit(active=False, reason="start_failed: " + str(exc), flight_commands=False)
