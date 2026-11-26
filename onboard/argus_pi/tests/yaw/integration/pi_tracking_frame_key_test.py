# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""실 C++ 추적 실행물의 프레임 계약 시험. FC/카메라 대신 loopback 합성 입력만 사용."""

import json
import socket
import subprocess
import sys
import threading
import time
import tempfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import zmq

# 2026-11-25 batch 2: tracking/tracks 는 팀 [이름][protobuf] (6375)
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tests/protobuf"))
import team_tracks_wire  # noqa: E402
import team_vision_wire  # noqa: E402  2026-11-25 batch 4: 6374 protobuf


def run(binary, root, output_path=None):
    uid = json.loads((root / "config/device/camera.json").read_text())["expected_uid"]
    state = {"epoch": 0, "requests": 0, "sample_ns": 0}

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            assert self.path == "/observations"
            state["requests"] += 1
            # 프레임 계약 시험에서는 합성 FC/영상 표본을 같은 시각에 만든다.
            # HTTP 서버 스레드 스케줄 지연은 FC 정렬 정확도 시험 대상이 아니다.
            stamp = state["sample_ns"]
            data = {
                "schema": "argus.fc.observations.v1",
                "clock_domain": "host_monotonic",
                "epoch": state["epoch"],
                "state": {
                    "uid": uid,
                    "identity_ok": True,
                    "fresh": True,
                    "local_ok": True,
                    "flow_quality": 100,
                },
                "sensors": {"flow_valid": True},
                "history": {},
            }
            samples = {
                "ATTITUDE": {"roll": 0, "pitch": 0, "yaw": 0},
                "LOCAL_POSITION_NED": {"x": 0, "y": 0, "z": -1.5},
                "DISTANCE_SENSOR": {
                    "orientation": 25,
                    "current_distance": 150,
                    "min_distance": 10,
                    "max_distance": 1000,
                },
            }
            for kind, sample in samples.items():
                data["history"][kind] = [{"received_monotonic_ns": stamp, "data": sample}]
            payload = json.dumps(data).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    input_port = pub.bind_to_random_port("tcp://127.0.0.1")
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        output_port = reservation.getsockname()[1]
    sub = context.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"tracking/tracks")
    sub.connect(f"tcp://127.0.0.1:{output_port}")
    temporary = tempfile.TemporaryDirectory(prefix="argus-frame-ids-")
    identity = Path(temporary.name) / "generation"
    subprocess.run(
        [str(binary), "--id-state-file", str(identity), "--initialize-id-state"],
        check=True,
        capture_output=True,
    )
    command = [
        str(binary),
        "--id-state-file",
        str(identity),
        "--calibration",
        str(root / "config/device/camera.json"),
        "--measurement",
        str(root / "config/tracking/measurement.json"),
        "--ucmc-profile",
        str(root / "config/tracking/ucmc.json"),
        "--expected-uid",
        uid,
        "--input",
        f"tcp://127.0.0.1:{input_port}",
        "--output",
        f"tcp://127.0.0.1:{output_port}",
        "--fc-http-port",
        str(server.server_port),
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    completed = False
    last_sent_ns = 0
    try:
        time.sleep(0.7)  # PUB/SUB subscription handshake, not freshness policy.

        def send(frame_id, source="camera-original", expected=True):
            nonlocal last_sent_ns
            # 실 입력은 30Hz이고 PUB의 HWM=2는 의도적인 과부하 드롭 경계다.
            # 이 계약 시험이 무간격 재시작 버스트로 그 경계를 시험하지 않게 한다.
            remaining_ns = 33333334 - (time.monotonic_ns() - last_sent_ns)
            if remaining_ns > 0:
                time.sleep(remaining_ns / 1e9)
            stamp = time.monotonic_ns()
            interval_ms = (stamp - last_sent_ns) / 1e6 if last_sent_ns else None
            last_sent_ns = stamp
            state["sample_ns"] = stamp
            requests_before = state["requests"]
            pixels = {
                "schema": "argus.perception.observations.v2",
                "source": source,
                "frame_id": frame_id,
                "clock_domain": "host_monotonic",
                "coordinate_frame": "camera_pixel",
                "captured_monotonic_ns": stamp,
                "image_width": 640,
                "image_height": 480,
                "detections": [
                    {
                        "class_id": 2,
                        "label": "car",
                        "confidence": 0.9,
                        "x1": 250,
                        "y1": 220,
                        "x2": 390,
                        "y2": 330,
                    }
                ],
            }
            pub.send_multipart(team_vision_wire.observations_frame(pixels, frame_id))
            ready = sub.poll(1500 if expected else 150)
            print(
                f"FRAME source={source} frame_id={frame_id} interval_ms={interval_ms} "
                f"http_requests={state['requests'] - requests_before} received={bool(ready)} "
                f"wait_ms={(time.monotonic_ns() - stamp) / 1e6:.3f}",
                flush=True,
            )
            if not expected:
                assert not ready, "duplicate frame must not renew the track sample"
                return None
            assert ready, "C++ tracker failed to publish synthetic frame"
            topic, packed = sub.recv_multipart()
            result = team_tracks_wire.decode_tracks(packed)
            assert topic == b"tracking/tracks"
            assert (result["source"], result["frame_id"]) == (source, frame_id)
            assert result["fc_epoch"] == state["epoch"]
            assert result["sample_monotonic_ns"] == pixels["captured_monotonic_ns"]
            assert not result["exposure_time_valid"] and result["exposure_monotonic_ns"] is None
            return result

        first = send(100)
        second = send(103)
        third = send(104)
        fourth = send(105)
        assert first["generation"] == second["generation"] == third["generation"]
        assert third["tracks"] and fourth["tracks"]
        assert third["tracks"][0]["track_id"] == fourth["tracks"][0]["track_id"]
        if output_path is not None:
            output_path.write_text(
                json.dumps(fourth, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
        send(105, expected=False)
        restarted = send(1)
        assert restarted["generation"] == first["generation"] + 1
        state["epoch"] = 1
        new_fc = send(2)
        assert new_fc["generation"] == restarted["generation"] + 1
        new_source = send(3, "camera-other")
        assert new_source["generation"] == new_fc["generation"] + 1
        assert state["requests"] >= 8
        print(
            "PASS: original frame keys, gaps, duplicate rejection, stable track ID, "
            "camera restart, FC epoch, source reset, receipt clock; passive GET only"
        )
        completed = True
    finally:
        process.terminate()
        output, error = process.communicate(timeout=5)
        if not completed or process.returncode != 0:
            print(output, error, file=sys.stderr)
        pub.close(0)
        sub.close(0)
        context.term()
        server.shutdown()
        server.server_close()
        temporary.cleanup()


if __name__ == "__main__":
    run(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]) if len(sys.argv) > 3 else None)
