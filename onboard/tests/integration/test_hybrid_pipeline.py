import csv
import hashlib
import json
import os
from pathlib import Path
import re
import select
import signal
import subprocess
import sys
import time

import cv2
import msgpack
import numpy as np
import pytest
import zmq

ROOT = Path(__file__).resolve().parents[2]
BIN = ROOT / "build/bin/argus-onboard"
SDK = Path("/home/user/thessen_raw14_viewer/build/i3_raw14_capture")
ENV = dict(os.environ, PYTHONPATH=os.pathsep.join((str(ROOT / "python"), str(Path(__file__).parent))))


class Broker:
    def __init__(self):
        self.process = subprocess.Popen([sys.executable, "-m", "argus_workers.transport.BusProxy",
            "--publish-bind", "tcp://127.0.0.1:*", "--subscribe-bind", "tcp://127.0.0.1:*"],
            env=ENV, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if not select.select([self.process.stdout], [], [], 5)[0]:
            self.close()
            raise RuntimeError("proxy readiness timed out")
        line = self.process.stdout.readline()
        endpoints = re.findall(r"tcp://127\.0\.0\.1:\d+", line)
        if len(endpoints) != 2:
            self.close()
            raise RuntimeError(f"proxy startup failed: {line}")
        self.publish, self.subscribe = endpoints
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket.connect(self.subscribe)

    def close(self):
        if hasattr(self, "socket"):
            self.socket.close(linger=0)
            self.context.term()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()

    def drain(self):
        result = []
        while self.socket.poll(0, zmq.POLLIN):
            parts = self.socket.recv_multipart()
            result.append((parts[0].decode(), msgpack.unpackb(parts[1], raw=False), parts[2] if len(parts) == 3 else None))
        return result


@pytest.fixture
def broker():
    instance = Broker()
    yield instance
    instance.close()


def video(path, frames=30):
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30, (640, 480))
    assert writer.isOpened()
    for index in range(frames):
        writer.write(np.full((480, 640, 3), 40 + index % 100, np.uint8))
    writer.release()


def command(broker, path, *extra):
    return [str(BIN), "--profiles", str(ROOT / "config/profiles.toml"), "--profile", "REPLAY",
        "--frame-source", "replay", "--camera", str(path), "--backend", "mock", "--python", sys.executable,
        "--labels", str(ROOT / "config/coco80.txt"), "--publish-endpoint", broker.publish,
        "--worker-timeout-ms", "2000", "--worker-startup-ms", "5000", "--log-every", "0", *extra]


def collect(process, broker, deadline=12, predicate=None):
    messages = []
    until = time.monotonic() + deadline
    while time.monotonic() < until:
        broker.socket.poll(30, zmq.POLLIN)
        messages.extend(broker.drain())
        if predicate and predicate(messages):
            return messages
        if process.poll() is not None:
            broker.socket.poll(100, zmq.POLLIN)
            messages.extend(broker.drain())
            return messages
    raise AssertionError("C++ pipeline did not finish/respond within deadline")


def stop_owned(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def test_archived_sources_and_canonical_entry():
    manifest = json.loads((ROOT / "legacy/python/manifest.json").read_text())
    assert len(manifest) >= 31
    for name, digest in manifest.items():
        path = ROOT / name
        assert path.name.endswith(".py.org")
        assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
    assert not (ROOT / "src/argus_vision").exists()
    package = (ROOT / "pyproject.toml").read_text()
    assert "argus_vision.vision_node" not in package
    assert 'where = ["python"]' in package
    launchers = list((ROOT / "scripts").glob("*.sh")) + list((ROOT / "deploy/systemd").glob("*.in"))
    for path in launchers:
        assert "argus_vision.vision_node" not in path.read_text()
        assert "-m argus_vision.busd" not in path.read_text()


def test_cpp_profile_and_missing_adapter_fail_explicitly():
    good = subprocess.run([str(BIN), "--profiles", str(ROOT / "config/profiles.toml"),
        "--profile", "REAL_IR", "--backend", "mock", "--check-config"], capture_output=True, text=True)
    assert good.returncode == 0 and "orchestrator=C++" in good.stdout and "queue_capacity=2" in good.stdout
    assert "capability=perception_only" in good.stdout
    bad = subprocess.run([str(BIN), "--profiles", str(ROOT / "config/profiles.toml"),
        "--profile", "SITL_IMAGE", "--backend", "mock", "--check-config"], capture_output=True, text=True)
    assert bad.returncode != 0 and "not implemented" in bad.stderr
    assert "mock fallback" not in bad.stdout


def test_cli_compatibility_executes_cpp():
    code = "from argus_workers.tools.OnboardLauncher import main; main()"
    result = subprocess.run([sys.executable, "-c", code, "--profiles", str(ROOT / "config/profiles.toml"),
        "--backend", "mock", "--check-config"], env=ENV, capture_output=True, text=True)
    assert result.returncode == 0 and "orchestrator=C++" in result.stdout


def test_proxy_multipart_and_sigterm(broker):
    publisher = broker.context.socket(zmq.PUB)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.connect(broker.publish)
    packet = [b"test/topic", msgpack.packb({"seq": 9}, use_bin_type=True), b"\x00JPEG\xff"]
    found = False
    until = time.monotonic() + 3
    try:
        while time.monotonic() < until:
            publisher.send_multipart(packet)
            if broker.socket.poll(30) and broker.socket.recv_multipart() == packet:
                found = True
                break
        assert found
    finally:
        publisher.close(linger=0)
    before = time.monotonic()
    broker.process.terminate()
    assert broker.process.wait(timeout=2) == 0
    assert time.monotonic() - before < 2


def test_cpp_replay_python_mock_observations_and_csv(tmp_path, broker):
    clip = tmp_path / "replay.avi"
    video(clip)
    csv_path = tmp_path / "latency.csv"
    process = subprocess.Popen(command(broker, clip, "--latency-csv", str(csv_path)), env=ENV,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        messages = collect(process, broker)
        assert process.wait(timeout=2) == 0, process.stderr.read()
    finally:
        stop_owned(process)
    observations = [header for topic, header, _ in messages if topic == "perception/observations"]
    assert observations
    ids = [item["frame_id"] for item in observations]
    assert ids == sorted(set(ids))
    for item in observations:
        assert item["schema"] == "argus.perception.observations.v2" and item["seq"] == item["frame_id"]
        assert len(item) == 23 and item["image_width"] == 640 and item["image_height"] == 480
        assert item["exposure_time_valid"] is False and item["exposure_monotonic_ns"] is None
        assert item["source_receive_monotonic_ns"] <= item["received_monotonic_ns"] <= item["published_monotonic_ns"]
        assert item["t_infer_us"] * 1000 <= item["published_monotonic_ns"]
        for box in item["detections"]:
            assert box["track_id"] is None and not any(name.startswith("hull") for name in box)
            assert 0 <= box["x1"] <= box["x2"] < 640 and 0 <= box["y1"] <= box["y2"] < 480
    rows = list(csv.DictReader(csv_path.open()))
    assert rows and all(float(row["total_ms"]) >= 0 for row in rows)


@pytest.mark.skipif(not SDK.exists(), reason="SDK capture executable not present")
def test_native_sdk_pattern_cpp_queue_python_ir_overlay_and_shutdown(tmp_path, broker):
    wrapper = tmp_path / "sdk-pattern.sh"
    wrapper.write_text(f"#!/bin/sh\nexec '{SDK}' --test-pattern --fps 10\n")
    wrapper.chmod(0o755)
    args = command(broker, "unused", "--profile", "REAL_IR", "--frame-source", "ir_sdk",
        "--ir-capture", str(wrapper), "--ir-timeout", "3", "--publish-preview")
    before_dirs = set(Path("/tmp").glob("argus-vision-*"))
    process = subprocess.Popen(args, env=ENV, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    children = []
    try:
        messages = collect(process, broker, predicate=lambda items:
            sum(topic == "perception/observations" for topic, _, _ in items) >= 5 and
            sum(topic == "vision/overlay/jpeg" for topic, _, _ in items) >= 3)
        child_file = Path(f"/proc/{process.pid}/task/{process.pid}/children")
        children = child_file.read_text().split()
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=12) == 0, process.stderr.read()
        messages.extend(broker.drain())
    finally:
        stop_owned(process)
    assert children and all(not Path(f"/proc/{pid}").exists() for pid in children)
    assert not set(Path("/tmp").glob("argus-vision-*")).difference(before_dirs)
    observations = {h["frame_id"]: h for topic, h, _ in messages if topic == "perception/observations"}
    previews = [(h, payload) for topic, h, payload in messages if topic == "vision/overlay/jpeg"]
    assert previews
    matched = 0
    for header, payload in previews:
        image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        assert image.shape == (480, 640, 3)
        assert header["frame_id"] == header["seq"] and header["session_id"]
        if header["frame_id"] not in observations:
            continue  # PUB is best effort; image/data delivery is not an atomic pair.
        item = observations[header["frame_id"]]
        assert item["exposure_time_valid"] is False and item["source_receive_monotonic_ns"] is not None
        box = item["detections"][0]
        b, g, r = map(int, image[box["y2"], (box["x1"] + box["x2"]) // 2])
        assert g > b + 30 and g > r + 30  # Same-frame full box is actually drawn in Pi JPEG.
        matched += 1
    assert matched


@pytest.mark.parametrize("module,expected", [("BadVisionWorker", "frame mismatch"), ("SlowVisionWorker", "timed out")])
def test_worker_failure_no_python_orchestration_fallback(tmp_path, broker, module, expected):
    clip = tmp_path / "replay.avi"
    video(clip, 4)
    before = set(Path("/tmp").glob("argus-vision-*"))
    process = subprocess.Popen(command(broker, clip, "--worker-module", module, "--worker-timeout-ms", "200"),
        env=ENV, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        _, error = process.communicate(timeout=15)
        assert process.returncode != 0 and expected in error
    finally:
        stop_owned(process)
    assert not set(Path("/tmp").glob("argus-vision-*")).difference(before)


def test_truncated_sdk_is_fault_not_normal_eof(tmp_path, broker):
    wrapper = tmp_path / "truncated-sdk.sh"
    wrapper.write_text("#!/bin/sh\nprintf I3R4\n")
    wrapper.chmod(0o755)
    process = subprocess.Popen(command(broker, "unused", "--profile", "REAL_IR", "--frame-source", "ir_sdk",
        "--ir-capture", str(wrapper), "--ir-timeout", "2"), env=ENV,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        _, error = process.communicate(timeout=10)
        assert process.returncode != 0 and "truncated SDK frame" in error
    finally:
        stop_owned(process)


@pytest.mark.skipif(not os.environ.get("ARGUS_HEF_SMOKE"), reason="optional real Hailo synthetic-video smoke")
def test_actual_hailo_synthetic_replay(tmp_path, broker):
    clip = tmp_path / "hailo-synthetic.avi"
    video(clip, 12)
    process = subprocess.Popen(command(broker, clip, "--backend", "hailo", "--hef", os.environ["ARGUS_HEF_SMOKE"],
        "--worker-timeout-ms", "10000", "--worker-startup-ms", "15000"), env=ENV,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        messages = collect(process, broker, deadline=25)
        assert process.wait(timeout=3) == 0, process.stderr.read()
    finally:
        stop_owned(process)
    observations = [h for topic, h, _ in messages if topic == "perception/observations"]
    assert observations and all(h["model"].endswith(".hef") for h in observations)
