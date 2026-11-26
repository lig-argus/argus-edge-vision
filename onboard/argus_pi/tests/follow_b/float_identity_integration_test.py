# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""실제 tracker 재시작/FC epoch 초기화와 float32 ID 왕복. 물리 장비 없음."""

import json
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
from process_integration_test import Onboard, wait


def run(root, build):
    uid = json.loads((root / "config/device/camera.json").read_text())["expected_uid"]
    fake = Onboard(uid)
    processes = []
    try:
        with tempfile.TemporaryDirectory(prefix="argus-float-process-") as folder:
            identity = Path(folder) / "generation"
            binary = build / "argus_follow_tracker"
            subprocess.run(
                [str(binary), "--id-state-file", str(identity), "--initialize-id-state"],
                check=True,
                capture_output=True,
            )
            # This regression isolates FC epoch allocation. Auto camera/FC transitions
            # have their own generation checks in camera_only_process_test.py.
            command = [
                str(binary),
                "--geometry",
                "fc-ground",
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
                fake.pixels,
                "--output",
                fake.tracks,
                "--fc-state-endpoint",
                fake.telemetry,
            ]

            def start():
                process = subprocess.Popen(
                    command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
                processes.append(process)
                return process

            def frame(generation):
                with fake.lock:
                    return next(
                        (
                            row
                            for row in reversed(fake.track_frames)
                            if row["generation"] == generation and row["tracks"]
                        ),
                        None,
                    )

            first_process = start()
            first = wait(lambda: frame(1), "first generation")
            assert first["tracks"][0]["track_id"] == 1
            fake.epoch += 1
            reset = wait(lambda: frame(2), "FC reset increments exactly one")
            assert reset["tracks"][0]["track_id"] != first["tracks"][0]["track_id"]
            first_process.send_signal(signal.SIGTERM)
            assert first_process.wait(5) == 0
            start()
            restarted = wait(lambda: frame(3), "persisted generation across real process restart")
            assert restarted["tracks"][0]["track_id"] == 1
            with fake.lock:
                for row in fake.track_frames:
                    for value in [row["generation"]] + [
                        track["track_id"] for track in row["tracks"]
                    ]:
                        assert 1 <= value <= 16777215
                        assert struct.unpack("<f", struct.pack("<f", value))[0] == value
                assert not fake.requests, "read-only tracker must never request FC control"
            processes[-1].send_signal(signal.SIGTERM)
            assert processes[-1].wait(5) == 0
            identity.write_text("ARGUS_FLOAT_IDS_V1\n16777215\n")
            exhausted = subprocess.run(command, capture_output=True, timeout=5)
            assert exhausted.returncode != 0 and b"generation exhausted" in exhausted.stderr
            assert identity.read_text() == "ARGUS_FLOAT_IDS_V1\n16777215\n"
            identity.unlink()
            missing = subprocess.run(command, capture_output=True, timeout=5)
            assert missing.returncode != 0 and b"state missing" in missing.stderr
            print(
                json.dumps(
                    dict(
                        passed=True,
                        generations=[1, 2, 3],
                        restart_track_id=1,
                        exhaustive_float_unit_separate=True,
                        missing_and_exhausted_fail_closed=True,
                        devices=False,
                    )
                )
            )
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        fake.close()


if __name__ == "__main__":
    run(Path(sys.argv[1]), Path(sys.argv[2]))
