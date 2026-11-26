# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""All FC messages mocked: no real mode/arming/flight commands."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "apps/real_fc"))
from tools.real_fc.vehicle import FlightLink, M, MODES
from tools.real_fc.config import EXPECTED_UID, INDOOR_PROFILE


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        with patch.object(FlightLink, "_run", return_value=None):
            self.v = FlightLink(clock=lambda: self.now)
        self.v.uid = EXPECTED_UID
        self.mav = Mock()
        self.v.link = SimpleNamespace(mav=self.mav, close=lambda: None)
        self.v.params = dict(
            INDOOR_PROFILE, EKF2_OF_QMIN=1, COM_POS_FS_EPH=5.0, SENS_FLOW_MAXHGT=2.0
        )
        self.seed()

    def seed(self, mode="hold", armed=True, ground=False):
        main, sub = MODES[mode]
        self.v.samples = dict(
            HEARTBEAT=(
                dict(base_mode=128 if armed else 0, custom_mode=main << 16 | sub << 24),
                self.now,
            ),
            EXTENDED_SYS_STATE=(dict(landed_state=1 if ground else 2), self.now),
            ESTIMATOR_STATUS=(dict(flags=79, pos_horiz_accuracy=0.1), self.now),
            LOCAL_POSITION_NED=(dict(x=0.0, y=0.0, z=-1.7, vx=0.0, vy=0.0, vz=0.0), self.now),
            ATTITUDE=(dict(roll=0.0, pitch=0.0, yaw=0.0), self.now),
            DISTANCE_SENSOR=(
                dict(current_distance=170, min_distance=8, max_distance=400),
                self.now,
            ),
            OPTICAL_FLOW_RAD=(dict(quality=50), self.now),
        )

    def tearDown(self):
        self.v.command_owner.close()

    def test_ground_or_uid_or_external_offboard_never_handoff(self):
        for mode, armed, ground, uid in (
            ("hold", False, True, EXPECTED_UID),
            ("hold", True, False, "0" * 36),
            ("offboard", True, False, EXPECTED_UID),
        ):
            self.seed(mode, armed, ground)
            self.v.uid = uid
            with self.assertRaises(ValueError):
                self.v.yaw_handoff()
        self.mav.command_long_send.assert_not_called()

    def test_actual_hold_releases_cached_writer_without_a_mode_command(self):
        self.v.external = self.v.owned_flight = True
        self.v.local_target = dict(stale=True)
        response = self.v.yaw_handoff()
        self.assertTrue(response["accepted"])
        self.assertTrue(response["ready"])
        self.assertFalse(self.v.external)
        self.assertFalse(self.v.owned_flight)
        self.assertIsNone(self.v.local_target)
        self.mav.command_long_send.assert_not_called()

    def test_offboard_without_prepared_owner_keeps_existing_goal(self):
        self.seed(mode="offboard")
        self.v.external = self.v.owned_flight = True
        self.v.command_owner.acquire()
        goal = dict(x=0.0, y=0.0, start_z=-1.7, distance=0.0, climb_at=None, yaw=0.3)
        self.v.local_target = goal
        with self.assertRaises(ValueError):
            self.v.yaw_handoff()
        self.assertIs(self.v.local_target, goal)
        self.assertTrue(self.v.external)
        self.mav.command_long_send.assert_not_called()

    def test_cached_setpoint_cannot_pass_a_busy_kernel_owner(self):
        self.v.command_owner.acquire = Mock(return_value=False)
        with self.assertRaises(ValueError):
            self.v._zero()
        self.mav.set_position_target_local_ned_send.assert_not_called()

    def test_observation_query_does_not_send_or_refresh_gui_lease(self):
        before = self.v.lease_at
        value = self.v.observations()
        self.assertEqual(value["timestamp_basis"], "host_receive")
        self.assertEqual(value["schema"], "argus.fc.observations.v1")
        self.assertEqual(self.v.lease_at, before)
        self.mav.command_long_send.assert_not_called()
        self.mav.set_position_target_local_ned_send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
