# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Read-only ground recorder. SUB input + HTTP status only, never flight commands."""

from pathlib import Path
import argparse, hashlib, json, os, select, signal, subprocess, time, urllib.request
import zmq

ROOT = Path(__file__).resolve().parents[4]
STOP = False


def load_pb_json():
    """2026-11-25 batch 2: tracking/tracks is team [name][protobuf] (src/argus/transport/pb_json.py)."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "argus_pb_json", ROOT / "src/argus/transport/pb_json.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def safe_state(state, status):
    return (
        state.get("fresh") is True
        and state.get("identity_ok") is True
        and state.get("armed") is False
        and state.get("ground") is True
        and not state.get("busy")
        and not state.get("owned_flight")
        and not state.get("external")
        and status.get("active") is False
        and status.get("bench_active") is False
        and not (state.get("follow_b") or {}).get("owner_active")
        and not (state.get("follow_b") or {}).get("bench_active")
        and not (state.get("manual_yaw") or {}).get("active")
    )


def key_tracks(raw):
    return (raw["source"], raw["frame_id"], raw["sample_monotonic_ns"])


def key_image(meta):
    return (meta["source"], meta["frame_id"], meta["received_monotonic_ns"])


def http(path, body=None):
    # Only these read-only endpoints may be used by the recorder.
    if path not in ("/state", "/vmy"):
        raise ValueError("read-only endpoint")
    if path == "/vmy" and (body or {}).get("action") != "status":
        raise ValueError("read-only action")
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(
        "http://127.0.0.1:8768" + path, data=data, headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=0.5) as response:
        return json.load(response)


def write_status(folder, **value):
    p = folder / "status.json"
    tmp = p.with_suffix(".next")
    tmp.write_text(json.dumps(value, ensure_ascii=False), encoding="utf8")
    os.replace(tmp, p)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--tracks", required=True)
    parser.add_argument("--images", required=True)
    parser.add_argument("--mode", choices=["moving", "yaw"], required=True)
    a = parser.parse_args()
    folder = a.output
    folder.mkdir(exist_ok=False, parents=True)
    profile = a.profile.read_bytes()
    (folder / "profile.config").write_bytes(profile)
    config = ROOT / "config"
    manifest = {
        str(p.relative_to(config)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in config.rglob("*")
        if p.is_file()
    }
    (folder / "manifest.json").write_text(
        json.dumps(
            dict(
                profile=str(a.profile),
                profile_sha256=hashlib.sha256(profile).hexdigest(),
                config_sha256=manifest,
                frame_key=["source", "frame_id", "sample_monotonic_ns"],
                shadow_only=True,
                flight_commands=False,
            ),
            indent=2,
        )
    )
    stderr = (folder / "native.stderr.log").open("w")
    child = subprocess.Popen(
        [str(Path(a.binary)), str(a.profile)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=stderr,
        text=True,
        bufsize=1,
    )
    pb_json = load_pb_json()
    context = zmq.Context()
    tracks = context.socket(zmq.SUB)
    images = context.socket(zmq.SUB)
    for sock, endpoint, topic in [
        (tracks, a.tracks, b"tracking/tracks"),
        (images, a.images, b"vision/overlay/jpeg"),
    ]:
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVHWM, 32)
        sock.connect(endpoint)
        sock.setsockopt(zmq.SUBSCRIBE, topic)
    poll = zmq.Poller()
    poll.register(tracks, zmq.POLLIN)
    poll.register(images, zmq.POLLIN)
    records = (folder / "frames.jsonl").open("w", encoding="utf8")
    events = (folder / "events.jsonl").open("w", encoding="utf8")
    last_event = None
    pending = {}
    pictures = {}
    state = {}
    status = {}
    last_poll = 0.0
    started = time.monotonic()
    count = 0
    paired = 0
    size = 0
    last_reason = "waiting_for_selection"

    def save(k, row, picture=None):
        nonlocal count, paired, size
        count += 1
        row["image"] = None
        row["image_missing_reason"] = "matching_frame_not_received"
        if picture:
            meta, jpeg = picture
            name = f"{count:06d}_{k[1]}.jpg"
            (folder / name).write_bytes(jpeg)
            size += len(jpeg)
            paired += 1
            row.update(
                image=name,
                image_sha256=hashlib.sha256(jpeg).hexdigest(),
                image_metadata=meta,
                image_missing_reason=None,
            )
        payload = json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n"
        records.write(payload)
        records.flush()
        size += len(payload.encode())

    try:
        while not STOP and time.monotonic() - started < 600 and size < 500 * 1024 * 1024:
            now = time.monotonic()
            if now - last_poll >= 0.2:
                last_poll = now
                try:
                    state = http("/state")
                    status = http("/vmy", dict(action="status", expected_uid=state["expected_uid"]))
                    if (
                        status.get("accepted") is not True
                        or status.get("identity_ok") is not True
                        or status.get("connected") is not True
                        or status.get("connected_uid") != state.get("expected_uid")
                        or status.get("expected_uid") != state.get("expected_uid")
                        or status.get("fc_epoch") != (state.get("follow_b") or {}).get("fc_epoch")
                        or status.get("mode") != a.mode
                    ):
                        raise ValueError("controller_identity_or_mode_mismatch")
                except Exception as e:
                    state = {}
                    status = {}
                    last_reason = "state_unavailable: " + str(e)
                if not safe_state(state, status):
                    for k, (_, row) in list(pending.items()):
                        save(k, row, pictures.get(k))
                    pending.clear()
                    pictures.clear()
                    last_reason = "paused_not_verified_disarmed_ground_idle"
                    child.stdin.write(json.dumps(dict(safety={})) + "\n")
                    child.stdin.flush()
                    if not select.select([child.stdout], [], [], 1)[0]:
                        raise RuntimeError("native response timeout")
                    child.stdout.readline()
                event = (
                    safe_state(state, status),
                    status.get("selected"),
                    status.get("selection_revision"),
                )
                if event != last_event:
                    events.write(
                        json.dumps(
                            dict(
                                at_ns=time.monotonic_ns(),
                                safe=event[0],
                                selection=event[1],
                                selection_revision=event[2],
                                reason=last_reason,
                            )
                        )
                        + "\n"
                    )
                    events.flush()
                    last_event = event
                write_status(
                    folder,
                    active=True,
                    records=count,
                    paired_images=paired,
                    bytes=size,
                    reason=last_reason,
                    output=str(folder),
                    flight_commands=False,
                )
            if os.statvfs(folder).f_bavail * os.statvfs(folder).f_frsize < 256 * 1024 * 1024:
                raise RuntimeError("free_space_limit")
            ready = dict(poll.poll(20))
            if images in ready:
                parts = images.recv_multipart()
                if (
                    len(parts) == 3
                    and parts[0] == b"vision/overlay/jpeg"
                    and len(parts[2]) < 2 * 1024 * 1024
                    and safe_state(state, status)
                    and time.monotonic() - last_poll < 0.5
                ):
                    # 2026-11-25 batch 4: 팀 vision 6374 protobuf (수와 1, 2, 4 ... 번째 로그는 pb_json)
                    try:
                        meta = pb_json.decode_pi_vision("vision/overlay/jpeg", parts[1])[0]
                    except ValueError:
                        meta = None
                    if meta is not None:
                        pictures[key_image(meta)] = (meta, parts[2])
            if tracks in ready:
                parts = tracks.recv_multipart()
                received = time.monotonic_ns()
                if (
                    len(parts) == 2
                    and parts[0] == b"tracking/tracks"
                    and len(parts[1]) < 1024 * 1024
                    and safe_state(state, status)
                    and time.monotonic() - last_poll < 0.5
                ):
                    try:
                        raw = pb_json.decode_pi_tracks(parts[1])
                    except ValueError as e:
                        last_reason = "tracks_decode_failed: " + str(e)
                        continue
                    selected = status.get("selected")
                    if selected:
                        selected = dict(
                            selected,
                            selection_revision=status.get("selection_revision"),
                            controller_session=status.get("controller_session"),
                        )
                        safety = {
                            k: state.get(k)
                            for k in [
                                "fresh",
                                "identity_ok",
                                "expected_uid",
                                "armed",
                                "ground",
                                "busy",
                            ]
                        }
                        safety["follow_active"] = status.get("active")
                        if raw.get("fc_epoch") is not None and raw["fc_epoch"] != (
                            state.get("follow_b") or {}
                        ).get("fc_epoch"):
                            last_reason = "fc_epoch_mismatch"
                            continue
                        request = dict(
                            safety=safety,
                            selection=selected,
                            tracks=raw,
                            received_ns=received,
                            safety_checked_ns=int(last_poll * 1e9),
                        )
                        child.stdin.write(json.dumps(request, allow_nan=False) + "\n")
                        child.stdin.flush()
                        if not select.select([child.stdout], [], [], 1)[0]:
                            raise RuntimeError("native response timeout")
                        result = json.loads(child.stdout.readline())
                        last_reason = result.get("error") or result.get("reason", "recording")
                        pending[key_tracks(raw)] = (
                            time.monotonic(),
                            dict(input=request, diagnostic=result),
                        )
                    else:
                        last_reason = "waiting_for_gui_selection"
            for k, (at, row) in list(pending.items()):
                if k in pictures:
                    save(k, row, pictures.pop(k))
                    del pending[k]
                elif time.monotonic() - at > 1:
                    save(k, row)
                    del pending[k]
            while len(pictures) > 64:
                del pictures[next(iter(pictures))]
            while len(pending) > 32:
                k = next(iter(pending))
                save(k, pending.pop(k)[1])
        last_reason = "stopped_or_recording_limit"
    except Exception as e:
        last_reason = "stopped: " + str(e)
    finally:
        for k, (_, row) in pending.items():
            save(k, row, pictures.get(k))
        records.close()
        events.close()
        tracks.close()
        images.close()
        context.term()
        child.stdin.close()
        try:
            child.wait(2)
        except subprocess.TimeoutExpired:
            child.terminate()
            child.wait(2)
        stderr.close()
        write_status(
            folder,
            active=False,
            records=count,
            paired_images=paired,
            bytes=size,
            reason=last_reason,
            output=str(folder),
            flight_commands=False,
        )


def stop(*_):
    global STOP
    STOP = True


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    main()
