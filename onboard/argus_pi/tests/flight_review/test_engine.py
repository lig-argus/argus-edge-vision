# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
from pathlib import Path
import copy, json, sys, tempfile, threading, urllib.request, urllib.error, unittest
import numpy as np
from pyulog import ULog

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT.parents[2]
sys.path.insert(0, str(ROOT / "apps/flight_review"))
from engine import analyze, add_audit, report, json_bytes, body_velocity, add_yaw_rate_topics
from server import make_server


class ReviewTests(unittest.TestCase):
    def test_yaw_degrees_and_tilt_corrected_rate(self):
        rate = np.radians(5.0)
        topics = {
            "trajectory_setpoint#0": {
                "name": "trajectory_setpoint",
                "instance": 0,
                "t": [1.0, 1.1, 1.5],
                "fields": {"yawspeed": [rate, -rate, None]},
            },
            "vehicle_attitude#0/euler": {
                "name": "Euler",
                "t": [1.0],
                "fields": {"roll_deg": [0.0], "pitch_deg": [30.0]},
            },
            "vehicle_angular_velocity#0": {
                "name": "vehicle_angular_velocity",
                "t": [1.1, 1.5],
                "fields": {"xyz[1]": [0.0, 0.0], "xyz[2]": [rate, rate]},
            },
        }
        add_yaw_rate_topics(topics)
        values = topics["trajectory_setpoint#0/yaw_rate_deg"]["fields"]["requested_yaw_deg_s"]
        np.testing.assert_allclose(values[:2], [5, -5])
        self.assertIsNone(values[2])
        actual = topics["vehicle_angular_velocity#0/yaw_rate_deg"]["fields"]
        self.assertAlmostEqual(actual["actual_heading_deg_s"][0], 5 / np.cos(np.radians(30)))
        self.assertIsNone(actual["actual_heading_deg_s"][1])
        self.assertAlmostEqual(actual["body_yaw_deg_s"][0], 5)

    def test_body_rotation_uses_actual_full_attitude_and_rejects_stale(self):
        half = np.sqrt(0.5)
        for q, v, expected in [
            ([1, 0, 0, 0], [1, 2, 3], [1, 2, 3]),
            ([half, 0, 0, half], [0, 1, 0], [1, 0, 0]),
            ([half, 0, half, 0], [0, 0, 1], [-1, 0, 0]),
            ([half, half, 0, 0], [0, 0, 1], [0, 1, 0]),
        ]:
            actual, age = body_velocity(
                np.array([1.1]), np.array([v]), np.array([1.0]), np.array([q])
            )
            np.testing.assert_allclose(actual[0], expected, atol=1e-12)
        actual, age = body_velocity(
            np.array([0.9, 1.21, 1.1]),
            np.array([[1, 2, 3], [1, 2, 3], [1, np.nan, 3]]),
            np.array([1.0]),
            np.array([[1, 0, 0, 0]]),
        )
        self.assertTrue(np.isnan(actual).all())

    @classmethod
    def setUpClass(cls):
        cls.path = PROJECT / "output/yaw_button_flicker_20261019/sess120_log101.ulg"
        if not cls.path.exists():
            raise unittest.SkipTest("Local preserved ULog fixture unavailable")
        cls.result = analyze(cls.path)

    def test_all_numeric_samples_and_instances_match_source(self):
        u = ULog(str(self.path))
        for ds in u.data_list:
            if "timestamp" not in ds.data:
                continue
            actual = self.result["topics"][f"{ds.name}#{ds.multi_id}"]
            order = np.argsort(ds.data["timestamp"], kind="stable")
            np.testing.assert_array_equal(actual["t"], ds.data["timestamp"][order] / 1e6)
            for k, v in ds.data.items():
                if k == "timestamp" or v.dtype.kind not in "biuf":
                    continue
                expected = v[order].astype(float)
                expected[~np.isfinite(expected)] = np.nan
                np.testing.assert_allclose(
                    np.array(actual["fields"][k], dtype=float),
                    expected,
                    rtol=0,
                    atol=0,
                    equal_nan=True,
                )
                self.assertEqual(
                    actual["meta"][k]["positive_infinity_indices"],
                    np.flatnonzero(np.isposinf(v[order])).tolist(),
                )
                self.assertEqual(
                    actual["meta"][k]["negative_infinity_indices"],
                    np.flatnonzero(np.isneginf(v[order])).tolist(),
                )

    def test_known_hold_rejection_and_motor_data(self):
        self.assertTrue(self.result["capabilities"]["actuator_motors"])
        self.assertEqual(sum("command 176" in x["text"] for x in self.result["findings"]), 2)
        self.assertFalse(self.result["capabilities"]["esc_status"])
        y = self.result["topics"]["trajectory_setpoint#0"]["fields"]["yawspeed"]
        self.assertFalse(any(v is not None and abs(v) > 1e-5 for v in y))

    def test_upward_altitude_preserves_raw_z_and_missing_targets(self):
        for topic, source, field in [
            ("vehicle_local_position#0", "z", "altitude_up_m"),
            ("trajectory_setpoint#0", "position[2]", "target_altitude_up_m"),
        ]:
            raw = self.result["topics"][topic]
            up = self.result["topics"][topic + "/up"]
            self.assertEqual(raw["t"], up["t"])
            self.assertTrue(up["derived"])
            for z, h in zip(raw["fields"][source], up["fields"][field]):
                if z is None:
                    self.assertIsNone(h)
                else:
                    self.assertEqual(h, -z)

    def test_report_safe_json_and_no_external_js(self):
        d = copy.deepcopy(self.result)
        d["file"] = "</script><script>alert(1)</script>"
        h = report(d)
        self.assertNotIn(d["file"], h)
        self.assertNotIn("<script src=", h)
        json.loads(json_bytes(d))

    def test_clock_epochs_are_separate_and_never_auto_confirmed(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "a.jsonl"
            rows = []
            for epoch, boot in [(0, 20), (1, 2)]:
                for i in range(3):
                    rows.append(
                        dict(
                            schema="argus.audit.v1",
                            session="a",
                            epoch=epoch,
                            mono_s=100 + i,
                            kind="clock_anchor",
                            data={"fc_boot_s": boot + i},
                        )
                    )
            p.write_text("\n".join(map(json.dumps, rows)))
            d = add_audit(copy.deepcopy(self.result), [p])
            self.assertEqual(len(d["alignment"]), 2)
            self.assertTrue(all(not x["confirmed_same_flight"] for x in d["alignment"]))

    def test_loopback_api_refuses_control_and_unauthorized_upload(self):
        s = make_server(0, self.result)
        t = threading.Thread(target=s.serve_forever, daemon=True)
        t.start()
        try:
            base = f"http://127.0.0.1:{s.server_port}"
            with urllib.request.urlopen(base) as r:
                html = r.read().decode()
            token = html.split("const TOKEN='")[1].split("'")[0]
            for route in ("/command", "/api/ulog"):
                req = urllib.request.Request(base + route, data=b"x")
                with self.assertRaises(urllib.error.HTTPError) as e:
                    urllib.request.urlopen(req)
                self.assertEqual(e.exception.code, 403)
            req = urllib.request.Request(base + "/api/data", headers={"X-Review-Token": token})
            with urllib.request.urlopen(req) as r:
                self.assertEqual(json.load(r)["sha256"], self.result["sha256"])
            for route, body, headers, code in [
                ("/command", b"{}", {"X-Review-Token": token}, 404),
                ("/api/ulog", b"not a log", {"X-Review-Token": token}, 400),
                ("/", None, {"Host": "untrusted.example"}, 403),
            ]:
                with self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(
                        urllib.request.Request(base + route, data=body, headers=headers)
                    )
                self.assertEqual(error.exception.code, code)
            with urllib.request.urlopen(req) as r:
                self.assertEqual(json.load(r)["sha256"], self.result["sha256"])
        finally:
            s.shutdown()
            s.server_close()
            t.join(2)
