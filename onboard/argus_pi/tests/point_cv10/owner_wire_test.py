# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Actual owner/MAVLink serializer, only mocked FlightLink. Never opens real devices."""

from pathlib import Path
import json, sys, unittest, math

wire = json.loads(Path(sys.argv[1]).read_text(encoding="utf8"))
root = (
    Path(sys.argv[2])
    if len(sys.argv) > 2
    else Path(__file__).resolve().parents[3] / "코드/main/ARGUS"
)
sys.argv = [sys.argv[0]]
sys.path.insert(0, str(root / "tests/real_fc"))
sys.path.insert(0, str(root / "apps/real_fc"))
sys.path.insert(0, str(Path(__file__).parent / "fixtures"))
import test_follow_b_owner as fixtures


class Cv10OwnerWire(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.BOwnerTests()
        self.f.setUp()
        self.addCleanup(self.f.tearDown)
        self.f.b.cv_gap_enabled = True
        self.f.acquire()
        self.f.seed(mode="offboard")
        self.shift = int(self.f.now * 1e9) - wire["normal"]["issued_monotonic_ns"]
        self.assertTrue(self.f.b.receive_setpoint(self.packet("normal")), self.f.b.last_reason)
        self.f.b.tick()
        self.f.mav.reset_mock()
        self.f.now += (
            wire["cv"]["issued_monotonic_ns"] - wire["normal"]["issued_monotonic_ns"]
        ) / 1e9
        self.f.seed(mode="offboard")

    def packet(self, kind, **extra):
        value = dict(wire[kind])
        for key in (
            "source_sample_monotonic_ns",
            "issued_monotonic_ns",
            "valid_until_monotonic_ns",
            "cv_origin_sample_ns",
            "cv_gap_deadline_monotonic_ns",
        ):
            if value.get(key):
                value[key] += self.shift
        value.update(extra)
        return self.f.velocity(**value)

    def test_cpp_cv_command_reaches_mock_mavlink(self):
        self.assertTrue(self.f.b.receive_setpoint(self.packet("cv")), self.f.b.last_reason)
        self.f.b.tick()
        args = self.f.mav.set_position_target_local_ned_send.call_args.args
        self.assertAlmostEqual(args[8], wire["cv"]["vx_m_s"])
        self.assertAlmostEqual(args[9], wire["cv"]["vy_m_s"])
        self.assertGreater(abs(args[8]) + abs(args[9]), 0)
        self.assertEqual(args[10], 0)
        self.f.mav.command_long_send.assert_not_called()

    def test_cannot_rejuvenate_origin_or_extend_deadline(self):
        origin = wire["cv"]["cv_origin_sample_ns"] + self.shift
        self.assertFalse(
            self.f.b.receive_setpoint(
                self.packet(
                    "cv",
                    source_sample_monotonic_ns=origin + 1,
                    cv_origin_sample_ns=origin + 1,
                    cv_gap_deadline_monotonic_ns=origin + 3000000001,
                )
            )
        )
        self.assertFalse(
            self.f.b.receive_setpoint(
                self.packet("cv", cv_gap_deadline_monotonic_ns=origin + 3100000000)
            )
        )
        self.f.mav.set_position_target_local_ned_send.assert_not_called()

    def test_expired_or_rc_command_never_sends(self):
        self.f.now = (wire["cv"]["cv_gap_deadline_monotonic_ns"] + self.shift) / 1e9
        self.f.seed(mode="offboard")
        now = int(self.f.now * 1e9)
        self.assertFalse(
            self.f.b.receive_setpoint(
                self.packet("cv", issued_monotonic_ns=now, valid_until_monotonic_ns=now + 50000000)
            )
        )
        self.f.b.ingest("STATUSTEXT", {"text": "Pilot took over using sticks"})
        self.assertFalse(self.f.b.receive_setpoint(self.packet("cv")))
        self.f.mav.set_position_target_local_ned_send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
