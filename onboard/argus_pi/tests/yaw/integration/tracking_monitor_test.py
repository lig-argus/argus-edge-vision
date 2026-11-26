# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""실제 감시 바이너리/버스 통합 검사. 합성 입력이며 카메라/FC를 열지 않는다."""

import json, os, sys, time, subprocess, tempfile, uuid, unittest
from pathlib import Path
import msgpack, zmq

# 2026-11-25 batch 2: tracking/tracks 는 팀 [이름][protobuf] (6375). 합성 tracks 는 tests/protobuf 도우미로 보낸다
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tests/protobuf"))
import team_tracks_wire  # noqa: E402
import team_vision_wire  # noqa: E402  2026-11-25 batch 4: 6374 protobuf

BIN = (
    Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else Path(os.environ["ARGUS_MONITOR_BIN"])
)
IMAGE_ENABLED = os.environ.get("ARGUS_MONITOR_IMAGE_ENABLED", "1") == "1"


class MonitorTest(unittest.TestCase):
    def setUp(self):
        self.ctx = zmq.Context()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.setsockopt(zmq.LINGER, 0)
        self.input_port = self.pub.bind_to_random_port("tcp://127.0.0.1")
        self.seq = 0
        self.frames = []
        self.status = self.ctx.socket(zmq.SUB)
        self.status.setsockopt(zmq.LINGER, 0)
        self.status.setsockopt(zmq.SUBSCRIBE, b"tracking/target_status")
        self.track = self.ctx.socket(zmq.SUB)
        self.track.setsockopt(zmq.LINGER, 0)
        self.track.setsockopt(zmq.SUBSCRIBE, b"tracking/tracks")
        self.view = self.ctx.socket(zmq.SUB)
        self.view.setsockopt(zmq.LINGER, 0)
        self.view.setsockopt(zmq.SUBSCRIBE, b"tracking/target_view")
        ports = []
        for _ in range(3):
            s = self.ctx.socket(zmq.PUB)
            ports.append(s.bind_to_random_port("tcp://127.0.0.1"))
            s.close(0)
        self.tracks, self.status_url, self.command = [f"tcp://127.0.0.1:{p}" for p in ports]
        self.status.connect(self.status_url)
        self.track.connect(self.tracks)
        self.view.connect(self.status_url)
        self.tmp = tempfile.TemporaryDirectory()
        self.log = (Path(self.tmp.name) / "monitor.log").open("w+")
        self.proc = None

    def tearDown(self):
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
                self.fail("monitor shutdown blocked")
            self.assertEqual(self.proc.returncode, 0, self.log.read())
        self.pub.close(0)
        self.status.close(0)
        self.track.close(0)
        self.view.close(0)
        self.ctx.term()
        self.log.close()
        self.tmp.cleanup()

    def launch(self, image=False):
        source = f"tcp://127.0.0.1:{self.input_port}"
        args = [
            str(BIN),
            "--input",
            "observations" if image else "tracks",
            "--tracks",
            self.tracks if image else source,
            "--status",
            self.status_url,
            "--command",
            self.command,
            "--max-age-ms",
            "120",
            "--max-gap-ms",
            "900",
        ]
        if image:
            args.extend(["--observations", source])
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=self.log)
        self.until(lambda s: s.get("flight_control") is False)
        time.sleep(0.15)

    def tracks_frame(self, ids=(1,), overlap=False):
        self.seq += 1
        now = time.monotonic_ns()
        tracks = [
            {
                "track_id": i,
                "class_id": 0,
                "label": "test_vehicle",
                "confidence": 0.9,
                "x1": 100.0,
                "y1": 150.0,
                "x2": 180.0,
                "y2": 220.0,
                "state": "CONFIRMED",
                "real_detection": True,
                "last_real_detection_monotonic_ns": now,
            }
            for i in ids
        ]
        frame = {
            "schema": "argus.tracking.tracks.v1",
            "source": "synthetic-pi-tracker",
            "frame_id": self.seq,
            "sample_monotonic_ns": now,
            "clock_domain": "host_monotonic",
            "timestamp_source": "host_receive_test",
            "exposure_monotonic_ns": None,
            "exposure_time_valid": False,
            "image_width": 640,
            "image_height": 480,
            "coordinate_frame": "camera_pixel",
            "generation": 1,
            "tracks": tracks,
        }
        self.pub.send_multipart(
            [b"tracking/tracks", team_tracks_wire.encode_tracks(frame, self.seq)]
        )
        self.frames.append(frame)

    def observe(self):
        self.seq += 1
        ns = time.monotonic_ns()
        us = time.time_ns()
        value = {
            "schema": "argus.perception.observations.v2",
            "source": "synthetic-pi-camera",
            "frame_id": self.seq,
            "seq": self.seq,
            "captured_unix_ns": us,
            "captured_monotonic_ns": ns,
            "published_monotonic_ns": ns,
            "image_width": 640,
            "image_height": 480,
            "model": "synthetic-test",
            "inference_ms": 1.0,
            "t_infer_us": 1,
            "t_publish_us": 1,
            "coordinate_frame": "camera_pixel",
            "observation_kind": "IMAGE2D",
            "detections": [
                {
                    "class_id": 0,
                    "label": "test_vehicle",
                    "confidence": 0.9,
                    "x1": 100.0,
                    "y1": 150.0,
                    "x2": 180.0,
                    "y2": 220.0,
                    "track_id": None,
                }
            ],
            "received_unix_ns": us,
            "received_monotonic_ns": ns,
            "exposure_monotonic_ns": None,
            "source_receive_monotonic_ns": ns,
            "clock_domain": "host_monotonic",
            "timestamp_source": "host_receive_test",
            "exposure_time_valid": False,
        }
        self.pub.send_multipart(team_vision_wire.observations_frame(value, self.seq))

    def until(self, predicate, pump=None, timeout=3):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pump:
                pump()
            if self.status.poll(30):
                parts = self.status.recv_multipart()
                state = msgpack.unpackb(parts[1], raw=False)
                self.assertFalse(state["flight_control"])
                if predicate(state):
                    return state
            else:
                time.sleep(0.01)
            if self.proc.poll() is not None:
                self.log.seek(0)
                self.fail(self.log.read())
        self.fail("monitor state timed out")

    def request(self, action, generation=0, track_id=0, request_id=None):
        with self.ctx.socket(zmq.REQ) as s:
            s.setsockopt(zmq.LINGER, 0)
            s.connect(self.command)
            value = {
                "schema": "argus.command.request.v1",
                "request_id": request_id or str(uuid.uuid4()),
                "action": action,
                "generation": generation,
                "track_id": track_id,
            }
            s.send(msgpack.packb(value, use_bin_type=True))
            self.assertTrue(s.poll(1500))
            return msgpack.unpackb(s.recv(), raw=False)

    def test_external_tracking_memory_and_no_flight_actions(self):
        self.launch()
        self.until(lambda s: s["sample_monotonic_ns"] > 0, self.tracks_frame)
        self.assertTrue(self.request("SELECT", 1, 1)["accepted"])
        self.assertTrue(self.request("START")["accepted"])
        locked = self.until(lambda s: s["target_phase"] == "LOCKED", self.tracks_frame)
        mission = locked["mission_target_id"]
        self.assertGreater(mission, 0)
        self.assertFalse(self.request("ARM")["accepted"])
        self.assertFalse(self.request("TAKEOFF")["accepted"])
        self.until(lambda s: s["target_phase"] == "GAP_HOLD")
        restored = self.until(
            lambda s: s["target_phase"] == "LOCKED" and s["track_id"] == 9,
            lambda: self.tracks_frame((9,)),
        )
        self.assertEqual(restored["mission_target_id"], mission)
        ambiguous = self.until(
            lambda s: s["target_phase"] != "LOCKED", lambda: self.tracks_frame((9, 10), True)
        )
        self.assertFalse(ambiguous["fresh_real_detection"])
        lost = self.until(lambda s: s["target_phase"] == "LOST", timeout=2)
        self.assertFalse(lost["fresh_real_detection"])
        self.assertTrue(self.request("STOP")["accepted"])
        self.until(lambda s: not s["monitoring"])

    @unittest.skipUnless(IMAGE_ENABLED, "explicit image ByteTrack disabled in this build")
    def test_existing_pi_observation_schema_runs_actual_bytetrack(self):
        self.launch(image=True)
        end = time.monotonic() + 3
        frame = None
        while time.monotonic() < end:
            self.observe()
            if self.track.poll(30):
                _, packed = self.track.recv_multipart()
                f = team_tracks_wire.decode_tracks(packed)
                if any(t["state"] == "CONFIRMED" and t["real_detection"] for t in f["tracks"]):
                    frame = f
                    break
        self.assertIsNotNone(frame)
        self.assertEqual(frame["source"], "synthetic-pi-camera")
        self.assertFalse(frame["exposure_time_valid"])
        tr = next(t for t in frame["tracks"] if t["real_detection"])
        self.assertEqual(tr["class_id"], 0)
        self.assertNotIn("ground", tr)
        self.assertEqual(tr["last_real_detection_monotonic_ns"], frame["sample_monotonic_ns"])
        self.observe()
        self.assertTrue(self.request("SELECT", frame["generation"], tr["track_id"])["accepted"])
        self.assertTrue(self.request("START")["accepted"])
        self.until(lambda s: s["target_phase"] == "LOCKED", self.observe)
        self.until(lambda s: s["target_phase"] == "LOST", timeout=2)
        self.assertTrue(self.request("STOP")["accepted"])

    def test_stale_original_samples_cannot_refresh_memory(self):
        self.launch()
        self.until(lambda s: s["sample_monotonic_ns"] > 0, self.tracks_frame)
        self.assertTrue(self.request("SELECT", 1, 1)["accepted"])
        self.assertTrue(self.request("START")["accepted"])
        self.until(lambda s: s["target_phase"] == "LOCKED", self.tracks_frame)

        def stale():
            f = dict(self.frames[-1])
            self.seq += 1
            f["frame_id"] = self.seq
            stamp = time.monotonic_ns() - 300_000_000
            f["sample_monotonic_ns"] = stamp
            f["tracks"] = [dict(f["tracks"][0], last_real_detection_monotonic_ns=stamp)]
            self.pub.send_multipart(
                [b"tracking/tracks", team_tracks_wire.encode_tracks(f, self.seq)]
            )

        lost = self.until(lambda s: s["target_phase"] == "LOST", stale, timeout=2)
        self.assertFalse(lost["fresh_real_detection"])
        self.assertFalse(lost["flight_control"])

    def until_view(self, predicate, pump=None, timeout=3):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pump:
                pump()
            if self.view.poll(30):
                _, payload = self.view.recv_multipart()
                v = msgpack.unpackb(payload, raw=False)
                self.assertEqual(v["schema"], "argus.tracking.target_view.v1")
                self.assertFalse(v["flight_control"])
                if predicate(v):
                    return v
        self.fail("object view timed out")

    def test_object_view_preserves_tracker_and_target_identity(self):
        self.launch()

        def two():
            self.seq += 1
            stamp = time.monotonic_ns()
            a = {
                "track_id": 7,
                "class_id": 0,
                "label": "test_vehicle",
                "confidence": 0.9,
                "x1": 100.0,
                "y1": 150.0,
                "x2": 180.0,
                "y2": 220.0,
                "state": "CONFIRMED",
                "real_detection": True,
                "last_real_detection_monotonic_ns": stamp,
            }
            b = dict(a, track_id=8, x1=300.0, x2=380.0)
            f = {
                "schema": "argus.tracking.tracks.v1",
                "source": "synthetic-pi-tracker",
                "frame_id": self.seq,
                "sample_monotonic_ns": stamp,
                "clock_domain": "host_monotonic",
                "timestamp_source": "host_receive_test",
                "exposure_monotonic_ns": None,
                "exposure_time_valid": False,
                "image_width": 640,
                "image_height": 480,
                "coordinate_frame": "camera_pixel",
                "generation": 1,
                "tracks": [a, b],
            }
            self.pub.send_multipart(
                [b"tracking/tracks", team_tracks_wire.encode_tracks(f, self.seq)]
            )
            self.frames.append(f)

        first = self.until_view(lambda v: len(v["objects"]) == 2, two)
        self.assertEqual([o["tracker_id"] for o in first["objects"]], [7, 8])
        self.assertTrue(all(o["target_id"] is None for o in first["objects"]))
        self.assertTrue(self.request("SELECT", 1, 7)["accepted"])
        pending = self.until_view(
            lambda v: v["selection_state"] == "PENDING" and v["target_id"] is not None, two
        )
        target = pending["target_id"]
        self.assertEqual(pending["objects"][0]["target_id"], target)
        self.assertIsNone(pending["objects"][1]["target_id"])
        self.assertFalse(pending["objects"][0]["binding_confirmed"])
        self.assertTrue(self.request("START")["accepted"])
        current = self.until_view(
            lambda v: v["objects"] and v["objects"][0]["binding_confirmed"], two
        )
        obj = current["objects"][0]
        self.assertEqual(obj["tracker_id"], 7)
        self.assertEqual(obj["target_id"], target)
        self.assertEqual(obj["class_id"], 0)
        self.assertEqual(obj["bbox"], [100.0, 150.0, 180.0, 220.0])
        self.assertEqual(
            obj["last_real_detection_monotonic_ns"], current["frame_key"]["sample_monotonic_ns"]
        )
        # 새 ID가 나타나도 현재 객체의 target 연결은 매니저가 확인한 뒤만 바뀐다.
        restored = self.until_view(
            lambda v: (
                v["objects"]
                and v["objects"][0]["tracker_id"] == 19
                and v["objects"][0]["binding_confirmed"]
            ),
            lambda: self.tracks_frame((19,)),
        )
        self.assertEqual(restored["objects"][0]["target_id"], target)
        lost = self.until_view(lambda v: v["target_phase"] == "LOST", timeout=2)
        self.assertEqual(lost["target_id"], target)
        self.assertFalse(lost["frame_available"])
        self.assertEqual(lost["objects"], [])
        self.assertTrue(self.request("STOP")["accepted"])
        stopped = self.until_view(
            lambda v: v["selection_state"] == "UNSELECTED" and v["target_id"] is None
        )
        self.assertGreater(stopped["command_epoch"], restored["command_epoch"])
        if os.environ.get("ARGUS_OBJECT_VIEW_EXAMPLE"):
            Path(os.environ["ARGUS_OBJECT_VIEW_EXAMPLE"]).write_text(
                json.dumps(current, ensure_ascii=False, indent=2) + "\n"
            )

    def test_unknown_cli_flight_options_are_rejected(self):
        r = subprocess.run([str(BIN), "--arm", "true"], capture_output=True, text=True, timeout=3)
        self.assertEqual(r.returncode, 2)


if __name__ == "__main__":
    unittest.main()
