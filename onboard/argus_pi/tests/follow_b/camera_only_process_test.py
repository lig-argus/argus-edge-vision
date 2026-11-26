# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Real UCMC process without FC, then loss/recovery of synthetic telemetry only."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import zmq
from process_integration_test import Onboard, endpoint
import team_tracks_wire  # tests/protobuf (process_integration_test 가 경로를 넣는다)
import team_vision_wire  # noqa: E402  2026-11-25 batch 4: 6374 protobuf

ROOT = Path(sys.argv[1]).resolve()
BUILD = Path(sys.argv[2]).resolve()
UID = json.loads((ROOT / "config/device/camera.json").read_text())["expected_uid"]
CAMERA_SOURCE = "ucmc_camera_only_static_calibration"


def start(folder, mode, pixels, output, telemetry):
    identity = folder / "generation"
    subprocess.run(
        [
            str(BUILD / "argus_follow_tracker"),
            "--id-state-file",
            str(identity),
            "--initialize-id-state",
        ],
        check=True,
        capture_output=True,
    )
    log = (folder / "tracker.log").open("w+")
    command = [
        str(BUILD / "argus_follow_tracker"),
        "--geometry",
        mode,
        "--calibration",
        str(ROOT / "config/device/camera.json"),
        "--camera-only-calibration",
        str(ROOT / "config/ucmc-camera-estimate.txt"),
        "--ucmc-profile",
        str(ROOT / "config/tracking/ucmc.json"),
        "--input",
        pixels,
        "--output",
        output,
        "--fc-state-endpoint",
        telemetry,
        "--id-state-file",
        str(identity),
    ]
    if mode != "camera":
        command += [
            "--measurement",
            str(ROOT / "config/tracking/measurement.json"),
            "--expected-uid",
            UID,
        ]
    return subprocess.Popen(command, stdout=log, stderr=log), log


def stop(process, log):
    process.terminate()
    try:
        process.wait(5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        raise AssertionError("tracker shutdown timed out")
    log.seek(0)
    contents = log.read()
    log.close()
    assert process.returncode == 0, contents
    return contents


def team_only_body(body):
    """pi_schema 없는 팀 생산자 관측 본문 (2026-11-25 batch 4: 받지 않아야 한다)."""
    message = team_vision_wire.observations_message(team_vision_wire.observation_body(body), 1)
    message.pi_schema = ""
    return message.SerializeToString()


def standalone(mode):
    context = zmq.Context()
    pixels = context.socket(zmq.PUB)
    pixels.linger = 0
    pixels.bind("tcp://127.0.0.1:*")
    source = pixels.getsockopt_string(zmq.LAST_ENDPOINT)
    trap = context.socket(zmq.XPUB)
    trap.linger = 0
    trap.bind("tcp://127.0.0.1:*")
    telemetry = trap.getsockopt_string(zmq.LAST_ENDPOINT)
    output = endpoint()
    receiver = context.socket(zmq.SUB)
    receiver.linger = 0
    receiver.subscribe(b"tracking/")
    receiver.connect(output)
    with tempfile.TemporaryDirectory(prefix="camera-only-") as tmp:
        process, log = start(Path(tmp), mode, source, output, telemetry)
        frames, statuses, stamps = [], [], {}

        def drain():
            while receiver.poll(0):
                topic, packed = receiver.recv_multipart()
                value = team_tracks_wire.decode(topic, packed)
                (frames if topic == b"tracking/tracks" else statuses).append(value)

        try:
            time.sleep(0.3)
            for i in range(80):
                assert process.poll() is None
                stamp = time.monotonic_ns()
                stamps[i] = stamp
                body = dict(
                    schema="argus.perception.observations.v2",
                    source="camera-only-fixture",
                    frame_id=i,
                    captured_monotonic_ns=stamp,
                    clock_domain="host_monotonic",
                    coordinate_frame="camera_pixel",
                    image_width=640,
                    image_height=480,
                    detections=[
                        dict(
                            class_id=0,
                            label="test_vehicle",
                            confidence=0.9,
                            x1=0.0,
                            y1=172.0,
                            x2=419.0,
                            y2=344.0,
                        )
                    ],
                )
                pixels.send_multipart(team_vision_wire.observations_frame(body))
                time.sleep(0.025)
                drain()
            if mode == "fc-ground":
                assert not frames, "strict FC-ground mode accepted input without FC"
            else:
                assert len(frames) > 20, len(frames)
                tracks = [f for f in frames if f["tracks"]]
                assert len(tracks) > 10
                assert len({f["tracks"][0]["track_id"] for f in tracks}) == 1
                for f in frames:
                    assert (
                        f["schema"] == "argus.tracking.tracks.v1"
                        and f["timestamp_source"] == CAMERA_SOURCE
                    )
                    assert not {"ground_pose", "fc_epoch", "motion_sample_ns"}.intersection(f)
                    assert f["sample_monotonic_ns"] == stamps[f["frame_id"]]
                    assert all("ground" not in t and "ground_anchor" not in t for t in f["tracks"])
                assert any(s.get("tracking_valid") and not s["cmc_valid"] for s in statuses)
            if mode == "camera":
                assert not trap.poll(0), "explicit camera mode subscribed to FC telemetry"
            # Old or future pixel timestamps must not become fresh preview tracks.
            drain()
            count = len(frames)
            for i, delta in [(900, -1_000_000_000), (901, 1_000_000_000)]:
                body.update(frame_id=i, captured_monotonic_ns=time.monotonic_ns() + delta)
                pixels.send_multipart(team_vision_wire.observations_frame(body))
                time.sleep(0.05)
                drain()
            assert not any(f["frame_id"] in (900, 901) for f in frames[count:])
            # 2026-11-25 batch 4: pi_schema 없는 팀 생산자 관측은 추적에 들어가지 않고, 거부 로그는
            # 1, 2, 4, 8 ... 번째만 남는다 (프레임마다 쌓이지 않는다)
            drain()
            count = len(frames)
            for i in range(10):
                body.update(frame_id=1000 + i, captured_monotonic_ns=time.monotonic_ns())
                pixels.send_multipart([b"perception/observations", team_only_body(body)])
                time.sleep(0.03)
                drain()
            assert not any(f["frame_id"] >= 1000 for f in frames[count:]), (
                "team-only observation tracked"
            )
            time.sleep(1.2)
            drain()
            assert (
                statuses
                and not statuses[-1].get("tracking_valid")
                and not statuses[-1]["cmc_valid"]
            )
            print(mode, "passed; frames", len(frames))
        finally:
            contents = stop(process, log)
            for s in [pixels, trap, receiver]:
                s.close()
            context.term()
        rejections = [
            line
            for line in contents.splitlines()
            if "team_wire vision: perception/observations missing_pi_schema" in line
        ]
        assert 1 <= len(rejections) <= 4, contents
        assert contents.count("pi_schema 가 없음") <= len(rejections) + 2, contents
        # protobuf 변환이 키를 버리거나 (PB_UNKNOWN_FIELD) 실패하지 않는다
        assert "PB_UNKNOWN_FIELD" not in contents and "pi_codec_table:" not in contents, contents


def real_producer():
    """실제 C++ 생산자 (VisionBusOwner, argus-onboard 와 같은 코드) 에서 실제 추적기로 (rv-b4-1 발견 5).

    vision_bus_probe serve 가 6374 모양의 임의 포트를 bind 하고 30 Hz 로 관측을 보낸다. 카메라 전용
    추적기가 그 관측으로 UCMC 추적과 객체 ID 를 낸다 (FC 없음, 합성 입력).
    """
    context = zmq.Context()
    source = endpoint()
    output = endpoint()
    trap = context.socket(zmq.XPUB)
    trap.linger = 0
    trap.bind("tcp://127.0.0.1:*")
    telemetry = trap.getsockopt_string(zmq.LAST_ENDPOINT)
    receiver = context.socket(zmq.SUB)
    receiver.linger = 0
    receiver.subscribe(b"tracking/")
    receiver.connect(output)
    probe = subprocess.Popen(
        [str(BUILD / "vision_bus_probe"), "serve", source, "4", "30", "0", "-", "-"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    with tempfile.TemporaryDirectory(prefix="camera-only-producer-") as tmp:
        process, log = None, None
        frames = []
        try:
            assert probe.stdout.readline().decode().startswith("PROBE_READY"), probe.stderr.read()
            process, log = start(Path(tmp), "camera", source, output, telemetry)
            until = time.monotonic() + 4.5
            while time.monotonic() < until and process.poll() is None:
                if receiver.poll(50):
                    topic, packed = receiver.recv_multipart()
                    if topic == b"tracking/tracks":
                        frames.append(team_tracks_wire.decode(topic, packed))
            assert process.poll() is None
            tracks = [f for f in frames if f["tracks"]]
            assert len(tracks) > 10, len(frames)
            assert len({f["tracks"][0]["track_id"] for f in tracks}) == 1
            assert all(f["timestamp_source"] == CAMERA_SOURCE for f in frames)
            assert not trap.poll(0), "explicit camera mode subscribed to FC telemetry"
            print("real producer passed; frames", len(frames), "tracked", len(tracks))
        finally:
            if process is not None:
                contents = stop(process, log)
                assert "missing_pi_schema" not in contents and "PB_UNKNOWN_FIELD" not in contents, (
                    contents
                )
            probe.terminate()
            probe.wait(5)
            for s in [trap, receiver]:
                s.close()
            context.term()


def transitions():
    onboard = Onboard(UID)
    onboard.silent = True
    with tempfile.TemporaryDirectory(prefix="camera-auto-transition-") as tmp:
        process, log = start(Path(tmp), "auto", onboard.pixels, onboard.tracks, onboard.telemetry)

        def wait_frame(schema, after_generation=0):
            until = time.monotonic() + 6
            while time.monotonic() < until:
                assert process.poll() is None
                with onboard.lock:
                    rows = list(onboard.track_frames)
                matching = [
                    f
                    for f in rows
                    if f["schema"] == schema and f["generation"] > after_generation and f["tracks"]
                ]
                if matching:
                    return matching[-1]
                time.sleep(0.03)
            raise AssertionError("missing transition " + schema + ": " + str(onboard.error))

        try:
            first = wait_frame("argus.tracking.tracks.v1")
            onboard.silent = False
            ready = wait_frame("argus.tracking.tracks.v5", first["generation"])
            assert ready["fc_epoch"] == onboard.epoch and ready["ground_pose"]
            assert all(t.get("ground") and t.get("ground_anchor") for t in ready["tracks"])
            onboard.silent = True
            lost = wait_frame("argus.tracking.tracks.v1", ready["generation"])
            assert lost["timestamp_source"] == CAMERA_SOURCE and "fc_epoch" not in lost
            assert not onboard.requests and not onboard.mode_requests and not onboard.setpoints
            print(
                "auto FC loss/recovery passed; generations",
                first["generation"],
                ready["generation"],
                lost["generation"],
            )
        finally:
            stop(process, log)
            onboard.close()


for mode in ["camera", "auto", "fc-ground"]:
    standalone(mode)
real_producer()
transitions()
print("Camera-only process checks passed; no devices or physical FC accessed")
