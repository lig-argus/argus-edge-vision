# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""새 B tracker 실행기의 실제 상태 PUB 검사. FC/카메라 데이터는 공급하지 않는다."""

import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import tempfile

import zmq

# 2026-11-25 batch 2: tracking/status 는 팀 [이름][protobuf] (6375)
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests/protobuf"))
import team_tracks_wire  # noqa: E402


def run(root, build_dir):
    temporary = tempfile.TemporaryDirectory(prefix="argus-status-ids-")
    identity = Path(temporary.name) / "generation"
    subprocess.run(
        [
            str(build_dir / "argus_follow_tracker"),
            "--id-state-file",
            str(identity),
            "--initialize-id-state",
        ],
        check=True,
        capture_output=True,
    )
    context = zmq.Context()
    pixels = context.socket(zmq.PUB)
    pixel_port = pixels.bind_to_random_port("tcp://127.0.0.1")
    telemetry = context.socket(zmq.PUB)
    fc_port = telemetry.bind_to_random_port("tcp://127.0.0.1")
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        output_port = reservation.getsockname()[1]
    status = context.socket(zmq.SUB)
    status.setsockopt(zmq.SUBSCRIBE, b"tracking/")
    status.connect(f"tcp://127.0.0.1:{output_port}")
    environment = dict(os.environ, ARGUS_FOLLOW_B_BUILD_DIR=str(build_dir))
    process = subprocess.Popen(
        [
            "bash",
            str(root / "run_follow_b_tracker.sh"),
            "--input",
            f"tcp://127.0.0.1:{pixel_port}",
            "--output",
            f"tcp://127.0.0.1:{output_port}",
            "--fc-state-endpoint",
            f"tcp://127.0.0.1:{fc_port}",
            "--id-state-file",
            str(identity),
        ],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        messages = []
        deadline = time.monotonic() + 6
        while len(messages) < 3 and time.monotonic() < deadline:
            assert process.poll() is None, "B tracker exited before status observation"
            if not status.poll(200):
                continue
            topic, packed = status.recv_multipart()
            assert topic == b"tracking/status", "onboard/input absent: tracks must not be emitted"
            message = team_tracks_wire.decode_status(packed)
            assert message["schema"] == "argus.tracking.status.v1"
            assert message["source"] == "pi-ucmc"
            assert message["fc_fresh"] is False and message["cmc_valid"] is False
            assert message["fps"] == 0
            assert isinstance(message["generation"], int) and message["generation"] > 0
            if messages:
                assert message["seq"] > messages[-1]["seq"]
                assert (
                    message["published_monotonic_ns"] - messages[-1]["published_monotonic_ns"]
                ) >= 1_000_000_000
                assert message["generation"] == messages[-1]["generation"]
            messages.append(message)
        assert len(messages) == 3, "no 1Hz tracking health stream received"
        print(
            json.dumps(
                {
                    "result": "PASS",
                    "messages": messages,
                    "fc_camera_input": False,
                    "tracks_received": 0,
                },
                ensure_ascii=False,
            )
        )
    finally:
        process.terminate()
        output, error = process.communicate(timeout=5)
        if process.returncode != 0:
            print(output, error, file=sys.stderr)
        for channel in (pixels, telemetry, status):
            channel.close(0)
        context.term()
        temporary.cleanup()


if __name__ == "__main__":
    run(Path(sys.argv[1]), Path(sys.argv[2]))
