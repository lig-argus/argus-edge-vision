# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""추적기 출력의 팀 6375 규약 시험 (2026-11-25 batch 2). 합성 관측만 쓰고 카메라, FC 를 열지 않는다.

1. --output 없이 실행한 추적기(카메라 전용)가 기본값 tcp://127.0.0.1:6375 에 bind 하고, 그 포트의
   listener 를 추적기 프로세스가 가진다 (게이트웨이 follow_service 의 준비 검사와 같은 판정).
   예전 6357 에는 bind 하지 않는다.
2. raw SUB 로 받은 메시지가 팀 프레임 형식이다: 정확히 2 프레임, 이름은 tracking/tracks 또는
   tracking/status, 본문은 protobuf (TrackingTracks, PiTrackingStatus), 팀 seq 는 토픽별 발행 번호
   (1 부터, 늘어남). tracks 의 pi_schema 는 v1 (카메라 전용), seq 키(pi_seq) 없음.
3. 고치지 않은 팀 ZmqBus 구독자(TeamTrackingBusProbe, pi_codec_table)가 같은 출력을 해석하고 해석
   실패가 없다.
6375 를 다른 프로세스가 쓰고 있으면 시험하지 않는다 (종료 코드 77, ctest SKIP).
"""

import importlib.util
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time

import zmq

ROOT = Path(sys.argv[1]).resolve()
BUILD = Path(sys.argv[2]).resolve()
sys.path.insert(0, str(ROOT / "tests/protobuf"))
import team_tracks_wire  # noqa: E402
import team_vision_wire  # noqa: E402  2026-11-25 batch 4: 6374 protobuf
from argus_msg import pi_tracking_status_pb2, tracking_tracks_pb2  # noqa: E402

TEAM_ENDPOINT = "tcp://127.0.0.1:6375"
TEAM_PORT = 6375
OLD_PORT = 6357


def load_supervisor():
    spec = importlib.util.spec_from_file_location(
        "argus_process_supervisor", ROOT / "src/argus/runtime/process_supervisor.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def port_free(port):
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def main():
    if not port_free(TEAM_PORT):
        print("SKIP: tcp 127.0.0.1:6375 is in use by another process")
        return 77
    supervisor = load_supervisor()
    context = zmq.Context()
    pixels = context.socket(zmq.PUB)
    pixels.linger = 0
    pixels.bind("tcp://127.0.0.1:*")
    source = pixels.getsockopt_string(zmq.LAST_ENDPOINT)
    raw = context.socket(zmq.SUB)
    raw.linger = 0
    raw.subscribe(b"tracking/")
    raw.connect(TEAM_ENDPOINT)
    with tempfile.TemporaryDirectory(prefix="team-tracking-bus-") as tmp:
        folder = Path(tmp)
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
        tracker = subprocess.Popen(
            [
                str(BUILD / "argus_follow_tracker"),
                "--geometry",
                "camera",
                "--calibration",
                str(ROOT / "config/device/camera.json"),
                "--camera-only-calibration",
                str(ROOT / "config/ucmc-camera-estimate.txt"),
                "--ucmc-profile",
                str(ROOT / "config/tracking/ucmc.json"),
                "--input",
                source,
                "--id-state-file",
                str(identity),
            ],
            stdout=log,
            stderr=log,
        )
        probe = None
        try:
            deadline = time.monotonic() + 5
            while not supervisor.listening(TEAM_PORT):
                assert tracker.poll() is None, "tracker exited before binding 6375"
                assert time.monotonic() < deadline, "tracker did not listen on 6375"
                time.sleep(0.05)
            assert supervisor.owns_listeners(tracker.pid, (TEAM_PORT,)), (
                "6375 listener is not owned by the tracker process"
            )
            assert not supervisor.listening(OLD_PORT), "old msgpack port 6357 is listening"
            # 6375 를 이미 가진 추적기가 있으면 두 번째 추적기는 bus 를 만들지 못해 시작하지 않는다
            second_identity = folder / "generation-second"
            subprocess.run(
                [
                    str(BUILD / "argus_follow_tracker"),
                    "--id-state-file",
                    str(second_identity),
                    "--initialize-id-state",
                ],
                check=True,
                capture_output=True,
            )
            second = subprocess.run(
                [
                    str(BUILD / "argus_follow_tracker"),
                    "--geometry",
                    "camera",
                    "--calibration",
                    str(ROOT / "config/device/camera.json"),
                    "--camera-only-calibration",
                    str(ROOT / "config/ucmc-camera-estimate.txt"),
                    "--ucmc-profile",
                    str(ROOT / "config/tracking/ucmc.json"),
                    "--input",
                    source,
                    "--id-state-file",
                    str(second_identity),
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
            assert second.returncode == 1, (second.returncode, second.stderr)
            assert "native Pi bus ownership failed" in second.stderr, second.stderr
            probe = subprocess.Popen(
                [str(BUILD / "TeamTrackingBusProbe"), TEAM_ENDPOINT, "20", "1", "8000"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            probe_out = {}

            def read_probe():
                probe_out["stdout"], probe_out["stderr"] = probe.communicate(timeout=15)

            reader = threading.Thread(target=read_probe)
            reader.start()
            time.sleep(0.3)
            raw_tracks, raw_status = [], []
            stop_at = time.monotonic() + 6
            frame_id = 0
            while time.monotonic() < stop_at and (len(raw_tracks) < 30 or not raw_status):
                assert tracker.poll() is None, "tracker exited"
                frame_id += 1
                stamp = time.monotonic_ns()
                body = dict(
                    schema="argus.perception.observations.v2",
                    source="team-bus-fixture",
                    frame_id=frame_id,
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
                while raw.poll(0):
                    parts = raw.recv_multipart()
                    assert len(parts) == 2, f"team frame must be [name][body], got {len(parts)}"
                    name, payload = parts
                    if name == team_tracks_wire.TRACKS:
                        message = tracking_tracks_pb2.TrackingTracks()
                        message.ParseFromString(payload)
                        raw_tracks.append(
                            (message.seq, team_tracks_wire.decode_tracks(payload), message)
                        )
                    elif name == team_tracks_wire.STATUS:
                        message = pi_tracking_status_pb2.PiTrackingStatus()
                        message.ParseFromString(payload)
                        raw_status.append((message.seq, team_tracks_wire.decode_status(payload)))
                    else:
                        raise AssertionError(f"unexpected topic {name!r}")
            reader.join(20)
            assert len(raw_tracks) >= 30, len(raw_tracks)
            assert raw_status, "no tracking/status on 6375"
            sequences = [seq for seq, _, _ in raw_tracks]
            assert sequences[0] >= 1 and all(b > a for a, b in zip(sequences, sequences[1:])), (
                sequences
            )
            status_sequences = [seq for seq, _ in raw_status]
            assert status_sequences[0] >= 1 and status_sequences == sorted(set(status_sequences))
            for _, value, message in raw_tracks:
                assert message.pi_schema == "argus.tracking.tracks.v1", message.pi_schema
                assert value["schema"] == "argus.tracking.tracks.v1" and "seq" not in value
                assert not message.HasField("pi_seq")
            assert any(value["tracks"] for _, value, _ in raw_tracks), "no UCMC tracks"
            for _, value in raw_status:
                assert value["schema"] == "argus.tracking.status.v1" and value["cmc_valid"] is False
            assert probe.returncode == 0, (
                probe_out.get("stdout", "")[-2000:],
                probe_out.get("stderr", ""),
            )
            lines = [json.loads(line) for line in probe_out["stdout"].splitlines() if line]
            done = lines[-1]
            assert done["done"] and done["decode_failures"] == 0 and done["tracks"] >= 20, done
            bus_tracks = [line for line in lines[:-1] if line["topic"] == "tracking/tracks"]
            bus_status = [line for line in lines[:-1] if line["topic"] == "tracking/status"]
            assert bus_status, "team ZmqBus subscriber got no tracking/status"
            raw_by_seq = {seq: value for seq, value, _ in raw_tracks}
            matched = [line for line in bus_tracks if line["sequence"] in raw_by_seq]
            assert matched, "team ZmqBus and raw SUB share no tracks sequence"
            for line in matched:
                assert line["value"] == raw_by_seq[line["sequence"]], line["sequence"]
                assert line["value"]["schema"] == "argus.tracking.tracks.v1"
            print(
                json.dumps(
                    dict(
                        result="PASS",
                        endpoint=TEAM_ENDPOINT,
                        raw_tracks=len(raw_tracks),
                        raw_status=len(raw_status),
                        first_tracks_seq=sequences[0],
                        last_tracks_seq=sequences[-1],
                        bus_tracks=len(bus_tracks),
                        bus_status=len(bus_status),
                        matched_sequences=len(matched),
                        devices=False,
                    )
                )
            )
        finally:
            if probe is not None and probe.poll() is None:
                probe.kill()
            tracker.terminate()
            try:
                tracker.wait(5)
            except subprocess.TimeoutExpired:
                tracker.kill()
                tracker.wait()
            log.seek(0)
            contents = log.read()
            log.close()
            pixels.close(0)
            raw.close(0)
            context.term()
            assert tracker.returncode == 0, contents
            # 2026-11-25 batch 2 리뷰: protobuf 변환이 키를 버리거나 (PB_UNKNOWN_FIELD) 실패하지 않는다
            assert "PB_UNKNOWN_FIELD" not in contents, contents
            assert "pi_codec_table:" not in contents, contents
    return 0


if __name__ == "__main__":
    sys.exit(main())
