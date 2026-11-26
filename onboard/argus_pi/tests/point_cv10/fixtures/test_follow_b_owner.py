# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Loopback B owner checks: MAVLink is always a Mock, no real FC access."""

import uuid
from test_yaw_handoff import HandoffTests
from tools.real_fc.follow_b_owner import FollowBOwner


class BOwnerTests(HandoffTests):
    def test_actual_hold_before_ack_retires_zero_without_cancelling_completion(self):
        self.acquire()
        self.seed(mode="offboard")
        hold = self.mode_wire("HOLD")
        self.assertEqual(self.b.mode_request(hold)["result"], "ACCEPTED")
        self.assertTrue(self.b.receive_setpoint(self.velocity(yaw_rate_rad_s=0.0)))
        self.b.tick()
        self.now += 0.3
        self.seed(mode="hold")
        self.b.tick()
        self.assertIsNotNone(self.b.pending, "expired zero cancelled actual Hold awaiting ACK")
        self.assertIsNotNone(self.b.token)
        self.assertIsNone(self.b.setpoint)
        self.b.ingest("COMMAND_ACK", dict(command=176, result=0))
        self.now += 0.01
        self.seed(mode="hold")
        self.b.tick()
        self.assertEqual(self.b.last_mode_event["event"], "MODE_COMPLETED")

    def test_hold_rpc_before_stop_zero_discards_previous_motion_cache(self):
        self.acquire()
        self.seed(mode="offboard")
        self.assertTrue(self.b.receive_setpoint(self.velocity(vx_m_s=0.2)))
        late_zero = self.velocity(command_epoch=2, track_id=0, revision=1, yaw_rate_rad_s=0.0)
        hold = self.mode_wire("HOLD", command_epoch=2, track_id=0, revision=1)
        self.assertEqual(self.b.mode_request(hold)["result"], "ACCEPTED")
        sends = self.mav.command_long_send.call_count
        # REQ/REP overtakes an earlier PUB zero. Dropping that old sequence is correct.
        self.assertFalse(self.b.receive_setpoint(late_zero))
        self.b.tick()
        self.assertIsNotNone(self.b.token, "old motion cache revoked accepted Hold")
        self.assertIsNotNone(self.b.pending, "old motion cache cancelled Hold result")
        self.assertEqual(self.mav.command_long_send.call_count, sends)
        self.assertIsNone(self.b.setpoint)
        self.assertFalse(self.b.receive_setpoint(self.velocity(vx_m_s=0.2)))
        self.b.ingest("COMMAND_ACK", dict(command=176, result=0))
        self.now += 0.01
        self.seed(mode="hold")
        self.b.tick()
        self.assertEqual(self.b.last_mode_event["event"], "MODE_COMPLETED")
        self.assertEqual(self.b.last_mode_event["command_id"], hold["command_id"])

    def test_hold_pending_new_zero_after_tick_entry_keeps_confirmation(self):
        from types import SimpleNamespace
        from unittest.mock import patch

        self.acquire()
        self.seed(mode="offboard")
        self.assertEqual(self.b.mode_request(self.mode_wire("HOLD"))["result"], "ACCEPTED")
        sends = self.mav.command_long_send.call_count
        polls = iter((True, False))

        def receive():
            # A valid packet arrives after tick() took its first clock sample.
            self.now += 0.001
            self.seed(mode="offboard")
            data = self.velocity(yaw_rate_rad_s=0.0)
            return [b"control/setpoint", self.b.pack.packb(data, use_bin_type=True)]

        channel = SimpleNamespace(poll=lambda _: next(polls, False), recv_multipart=receive)
        with patch.object(self.b, "sub", channel):
            self.b.tick()
        self.assertIsNotNone(self.b.token, "fresh zero falsely revoked Hold ownership")
        self.assertIsNotNone(self.b.pending, "fresh zero cancelled Hold confirmation")
        self.assertEqual(self.mav.command_long_send.call_count, sends)

    def prepare_direct(self):
        self.seed(mode="offboard")
        self.v.external = self.v.owned_flight = self.v.owned_mode_confirmed = True
        self.v.command_owner.acquire()
        self.v._gcs_active = lambda: True
        self.v.local_target = dict(
            x=0.0,
            y=0.0,
            start_z=-1.7,
            target_z=-1.7,
            distance=0.0,
            height=1.7,
            yaw=0.0,
            climb_at=self.now - 10,
        )
        self.goal = self.v.local_target
        self.assertTrue(self.v.yaw_handoff()["ready"])
        self.acquire()
        self.assertTrue(self.b.handoff_pending)

    def test_direct_handoff_commits_only_on_valid_egress(self):
        for speed in (0.0, 0.2):
            self.prepare_direct()
            self.assertIs(self.v.local_target, self.goal)
            self.assertTrue(self.v.external)
            self.b.tick()  # no command: original position sender must remain enabled
            self.assertTrue(self.v.external)
            self.assertTrue(self.b.receive_setpoint(self.velocity(vx_m_s=speed)))
            self.assertTrue(self.v.external)
            self.b.tick()
            self.assertFalse(self.v.external)
            self.assertIsNone(self.v.local_target)
            self.assertFalse(self.b.handoff_pending)
            self.mav.command_long_send.assert_not_called()
            self.b.invalidate("released")

    def test_prepare_release_and_failure_preserve_original_height(self):
        self.prepare_direct()
        self.b.request(self.req("release"))
        self.assertIs(self.v.local_target, self.goal)
        self.assertTrue(self.v.external)
        self.assertTrue(not self.v.command_owner.path or self.v.command_owner.held)
        self.acquire()
        self.assertEqual(self.b.request(self.req("hold"))["status"], "failed")
        self.assertIs(self.v.local_target, self.goal)
        self.assertTrue(self.v.external)
        self.mav.command_long_send.assert_not_called()

    def test_expired_first_command_never_replaces_hover(self):
        self.prepare_direct()
        self.assertTrue(self.b.receive_setpoint(self.velocity(vx_m_s=0.2)))
        self.now += 0.3
        self.seed(mode="offboard")
        self.b.tick()
        self.assertTrue(self.v.external)
        self.assertIs(self.v.local_target, self.goal)
        self.assertIsNone(self.b.token)
        self.mav.command_long_send.assert_not_called()
        self.mav.set_position_target_local_ned_send.assert_not_called()

    def test_handoff_timeout_keeps_hover_and_requires_new_admission(self):
        self.prepare_direct()
        self.now += 16
        self.seed(mode="offboard")
        self.b.tick()
        self.assertTrue(self.v.external)
        self.assertIs(self.v.local_target, self.goal)
        self.assertEqual(self.b.request(self.req("acquire"))["status"], "failed")
        self.mav.command_long_send.assert_not_called()

    def test_queued_operator_command_blocks_handoff_commit(self):
        self.prepare_direct()
        self.assertTrue(self.b.receive_setpoint(self.velocity(vx_m_s=0.2)))
        self.v.jobs.append(dict(action="land"))
        self.b.tick()
        self.assertIs(self.v.local_target, self.goal)
        self.assertTrue(self.v.external)
        self.assertIsNone(self.b.token)
        self.mav.set_position_target_local_ned_send.assert_not_called()

    def test_rc_override_during_preparation_never_sends_follow(self):
        self.prepare_direct()
        self.b.ingest("STATUSTEXT", {"text": "Pilot took over"})
        self.assertFalse(self.b.receive_setpoint(self.velocity(vx_m_s=0.2)))
        self.mav.set_position_target_local_ned_send.assert_not_called()

    def setUp(self):
        super().setUp()
        tag = uuid.uuid4().hex
        self.b = FollowBOwner(
            self.v, ("inproc://state-" + tag, "inproc://rpc-" + tag, "inproc://cmd-" + tag)
        )
        self.v.follow_b = self.b
        self.seq = 0

    def tearDown(self):
        self.b.close()
        super().tearDown()

    def req(self, action, **extra):
        self.seq += 1
        now = int(self.now * 1e9)
        d = dict(
            schema="argus.follow.rpc.v1",
            source="argus-follow-controller",
            request_id=str(self.seq),
            client_session="test",
            action=action,
            uid=self.v.expected_uid,
            clock_domain="host_monotonic",
            onboard_session=self.b.session,
            fc_epoch=self.v.pose_epoch,
            owner_token=self.b.token or "",
            command_epoch=1,
            generation=1,
            track_id=1,
            source_sample_monotonic_ns=now,
            bench=False,
            issued_monotonic_ns=now,
            valid_until_monotonic_ns=now + 200_000_000,
            seq=self.seq,
            revision=0,
        )
        d.update(extra)
        return d

    def acquire(self):
        r = self.b.request(self.req("acquire"))
        self.assertEqual(r["status"], "accepted", r)

    def velocity(self, **extra):
        d = self.req(
            "velocity",
            schema="argus.control.setpoint.v1",
            coordinate_frame="body_frd",
            vx_m_s=0.0,
            vy_m_s=0.0,
            vz_m_s=0.0,
            yaw_rate_rad_s=0.1,
        )
        d.update(extra)
        return d

    def test_acquire_does_not_change_fc_mode(self):
        self.acquire()
        self.mav.command_long_send.assert_not_called()
        self.mav.set_position_target_local_ned_send.assert_not_called()

    def test_fc_uid_ground_and_foreign_owner_closed(self):
        self.seed(ground=True)
        self.assertEqual(self.b.request(self.req("acquire"))["status"], "failed")
        self.seed()
        self.v.external = True
        self.assertEqual(self.b.request(self.req("acquire"))["reason"], "handoff_not_ready")

    def test_expired_and_wrong_session_closed(self):
        self.acquire()
        self.assertFalse(
            self.b.receive_setpoint(self.velocity(valid_until_monotonic_ns=int(self.now * 1e9) - 1))
        )
        self.assertFalse(self.b.receive_setpoint(self.velocity(onboard_session="old")))
        self.assertFalse(self.b.receive_setpoint(self.velocity(uid="0" * 36)))

    def test_velocity_is_sent_only_from_tick_and_expires(self):
        self.acquire()
        self.seed(mode="offboard")
        self.assertTrue(self.b.receive_setpoint(self.velocity()))
        self.mav.set_position_target_local_ned_send.assert_not_called()
        self.b.tick()
        self.assertAlmostEqual(self.mav.set_position_target_local_ned_send.call_args.args[-1], 0.1)
        n = self.mav.set_position_target_local_ned_send.call_count
        self.now += 0.3
        self.seed()
        self.b.tick()
        self.assertEqual(self.mav.set_position_target_local_ned_send.call_count, n)
        self.assertIsNone(self.b.setpoint)

    def test_replay_revoke_and_epoch_reject_old_command(self):
        d = self.req("acquire")
        a = self.b.request(d)
        self.assertEqual(a, self.b.request(d))
        d2 = dict(d, action="release")
        self.assertEqual(self.b.request(d2)["status"], "failed")
        old = self.velocity()
        self.b.request(self.req("revoke", next_revision=1))
        self.assertFalse(self.b.receive_setpoint(old))
        self.v.pose_epoch += 1
        self.b.tick()
        self.assertIsNone(self.b.token)

    def test_offboard_requires_actual_prestream_ack_and_mode(self):
        self.acquire()
        self.assertEqual(
            self.b.request(self.req("start_offboard"))["reason"], "prestream_not_ready"
        )
        self.b.prestream_at = int((self.now - 1.1) * 1e9)
        self.b.last_stream = int(self.now * 1e9)
        reply = self.b.request(self.req("start_offboard"))
        self.assertEqual(reply["status"], "pending")
        self.b.ingest("COMMAND_ACK", dict(command=176, result=0))
        self.b.tick()
        self.assertIsNotNone(self.b.pending)
        self.now += 0.01
        self.seed(mode="offboard")
        self.b.tick()
        self.assertIsNone(self.b.pending)

    def test_rc_takeover_revokes(self):
        self.acquire()
        self.b.ingest("STATUSTEXT", dict(text="Pilot took over using sticks"))
        self.assertIsNone(self.b.token)
        self.assertTrue(self.b.rc_override)

    def status(self, **extra):
        return (
            dict(
                schema="argus.follow.status.v1",
                source="argus-follow-controller",
                client_session="test",
                uid=self.v.expected_uid,
                onboard_session=self.b.session,
                fc_epoch=self.v.pose_epoch,
                clock_domain="host_monotonic",
                received_monotonic_ns=int(self.now * 1e9),
                seq=100,
                status={},
            )
            | extra
        )

    def test_status_watchdog_is_separate_from_scan_velocity_ttl(self):
        self.acquire()
        # Hold/Scan에는 속도 setpoint가 없어도 상태가 살아 있으면 제어권을 유지한다.
        for _ in range(4):
            self.now += 1
            self.seed()
            self.assertTrue(self.b.receive_status(self.status(seq=int(self.now))))
            self.b.tick()
            self.assertIsNotNone(self.b.token)
        self.mav.command_long_send.assert_not_called()
        self.now += 3.01
        self.seed()
        self.b.tick()
        self.assertIsNone(self.b.token)
        self.assertEqual(self.b.last_reason, "follow_status_timeout")

    def test_status_does_not_extend_setpoint_and_wrong_session_is_ignored(self):
        self.acquire()
        self.seed(mode="offboard")
        self.assertTrue(self.b.receive_setpoint(self.velocity()))
        self.b.tick()
        self.assertFalse(self.b.receive_status(self.status(client_session="other")))
        self.now += 0.21
        self.seed(mode="offboard")
        self.assertTrue(self.b.receive_status(self.status()))
        self.b.tick()
        self.assertIsNone(self.b.token)
        self.assertIsNone(self.b.setpoint)
        self.assertEqual(self.mav.set_position_target_local_ned_send.call_args.args[-1], 0)
        self.assertEqual(self.mav.command_long_send.call_args.args[4:7], (1, 4, 3))

    def test_status_timeout_never_overrides_rc(self):
        self.acquire()
        self.now += 3.01
        self.seed(mode="offboard")
        self.b.rc_override = True
        self.b.tick()
        self.assertIsNone(self.b.token)
        self.mav.command_long_send.assert_not_called()
        self.mav.set_position_target_local_ned_send.assert_not_called()

    def test_setpoint_rate_is_twenty_hz(self):
        self.acquire()
        self.seed(mode="offboard")
        self.assertTrue(self.b.receive_setpoint(self.velocity()))
        self.b.tick()
        self.now += 0.049
        self.seed(mode="offboard")
        self.b.tick()
        self.assertEqual(self.mav.set_position_target_local_ned_send.call_count, 1)
        self.now += 0.002
        self.seed(mode="offboard")
        self.b.tick()
        self.assertEqual(self.mav.set_position_target_local_ned_send.call_count, 2)

    def test_actual_rpc_socket_preserves_envelope_and_returns_owner(self):
        sock = self.b.context.socket(self.b.zmq.REQ)
        sock.setsockopt(self.b.zmq.LINGER, 0)
        sock.connect(self.b.rpc.getsockopt(self.b.zmq.LAST_ENDPOINT).decode())
        request = self.req("acquire")
        sock.send(self.b.pack.packb(request, use_bin_type=True))
        for _ in range(30):
            self.b.tick()
            if sock.poll(10):
                break
        self.assertTrue(sock.poll(100))
        response = self.b.pack.unpackb(sock.recv(), raw=False)
        self.assertEqual(response["request_id"], request["request_id"])
        self.assertEqual(response["onboard_session"], self.b.session)
        self.assertEqual(response["status"], "accepted")
        sock.close(0)

    def mode_wire(self, mode="OFFBOARD", **extra):
        action = {"OFFBOARD": "start_offboard", "HOLD": "hold", "HOLD_HEADING": "heading"}[mode]
        request = self.req(action)
        return (
            request
            | dict(
                schema="argus.control.mode_request.v1",
                command_id=request["request_id"],
                session_id=request["client_session"],
                mode=mode,
            )
            | extra
        )

    def test_pr26_mode_acceptance_event_and_actual_mode_are_separate(self):
        self.acquire()
        self.b.prestream_at = int((self.now - 1.1) * 1e9)
        self.b.last_stream = int(self.now * 1e9)
        request = self.mode_wire()
        reply = self.b.mode_request(request)
        self.assertEqual(reply["result"], "ACCEPTED")
        self.assertIsNone(self.b.last_mode_event)
        self.b.ingest("COMMAND_ACK", dict(command=176, result=0))
        self.b.tick()
        self.assertIsNotNone(self.b.pending)
        self.now += 0.01
        self.seed(mode="offboard")
        self.b.tick()
        self.assertEqual(self.b.last_mode_event["event"], "MODE_COMPLETED")
        self.assertEqual(self.b.last_mode_event["command_id"], request["command_id"])
        self.assertEqual(self.b.last_mode_event["requester_session_id"], request["session_id"])
        self.assertEqual(self.b.last_mode_event["actual_mode"], "OFFBOARD")

    def test_pr26_duplicate_optional_extensions_and_expiry(self):
        self.acquire()
        request = self.mode_wire("HOLD", future_optional=1)
        self.assertEqual(self.b.mode_request(request)["result"], "ACCEPTED")
        before = self.mav.command_long_send.call_count
        self.assertEqual(
            self.b.mode_request(request | dict(future_optional=2))["result"], "ACCEPTED"
        )
        self.assertEqual(self.mav.command_long_send.call_count, before)
        self.assertEqual(
            self.b.mode_request(self.mode_wire("HOLD", valid_until_monotonic_ns=1))["result"],
            "REJECTED",
        )
        self.assertEqual(self.mav.command_long_send.call_count, before)

    def test_pr26_rpc_multipart(self):
        self.acquire()
        sock = self.b.context.socket(self.b.zmq.REQ)
        sock.setsockopt(self.b.zmq.LINGER, 0)
        sock.connect(self.b.rpc.getsockopt(self.b.zmq.LAST_ENDPOINT).decode())
        request = self.mode_wire("HOLD")
        sock.send_multipart(
            [b"control/mode_request", self.b.pack.packb(request, use_bin_type=True)]
        )
        for _ in range(30):
            self.b.tick()
            if sock.poll(10):
                break
        self.assertTrue(sock.poll(100))
        topic, body = sock.recv_multipart()
        reply = self.b.pack.unpackb(body, raw=False)
        self.assertEqual(topic, b"control/mode_result")
        self.assertEqual(reply["schema"], "argus.control.mode_result.v1")
        self.assertEqual(reply["command_id"], request["command_id"])
        self.assertEqual(reply["session_id"], self.b.session)
        self.assertEqual(reply["result"], "ACCEPTED")
        sock.close(0)

    def test_pr26_timeout_event_and_original_time_are_preserved(self):
        self.acquire()
        self.b.mode_request(self.mode_wire("HOLD"))
        self.now += 2.01
        self.seed()
        self.b.tick()
        self.assertEqual(self.b.last_mode_event["event"], "MODE_FAILED")
        stamp = self.b.last_mode_event["received_monotonic_ns"]
        self.now += 0.11
        self.seed()
        self.b.tick()
        self.assertEqual(self.b.last_mode_event["received_monotonic_ns"], stamp)

    def test_pr26_transport_failure_is_not_acceptance(self):
        self.acquire()
        self.mav.command_long_send.side_effect = OSError("mock transport unavailable")
        reply = self.b.mode_request(self.mode_wire("HOLD"))
        self.assertEqual(reply["result"], "FAILED")
        self.assertIsNone(self.b.pending)

    def test_pr26_execution_deadline_is_not_completion_deadline(self):
        self.acquire()
        self.b.prestream_at = int((self.now - 1.1) * 1e9)
        self.b.last_stream = int(self.now * 1e9)
        request = self.mode_wire()
        self.assertEqual(self.b.mode_request(request)["result"], "ACCEPTED")
        self.b.ingest("COMMAND_ACK", dict(command=176, result=0))
        self.now += 0.4
        self.seed(mode="offboard")
        self.b.tick()
        self.assertIsNone(self.b.pending)
        self.assertEqual(self.b.last_mode_event["event"], "MODE_COMPLETED")
        self.assertGreater(
            self.b.last_mode_event["received_monotonic_ns"], request["valid_until_monotonic_ns"]
        )

    def test_pr26_session_alias_must_match(self):
        self.acquire()
        self.assertEqual(
            self.b.mode_request(self.mode_wire("HOLD", session_id="other"))["result"], "REJECTED"
        )
        self.assertFalse(self.b.receive_status(self.status(session_id="other")))

    def test_pr26_forward_setpoint_and_alias_conflict(self):
        self.acquire()
        self.seed(mode="offboard")
        request = self.velocity()
        for external, internal in (
            ("forward", "vx_m_s"),
            ("right", "vy_m_s"),
            ("down", "vz_m_s"),
            ("yaw_rate", "yaw_rate_rad_s"),
        ):
            request[external] = request.pop(internal)
        self.assertTrue(self.b.receive_setpoint(request))
        request = self.velocity(forward=99.0)
        self.assertFalse(self.b.receive_setpoint(request))

    def test_ground_bench_is_explicit_bounded_and_no_arm_mode(self):
        self.seed(ground=True, armed=False)
        body = dict(
            action="bench_yaw",
            expected_uid=self.v.expected_uid,
            props_removed=True,
            yaw_rate_deg_s=5,
            duration_s=5,
        )
        self.assertTrue(self.b.bench_request(body)["accepted"])
        self.b.tick()
        self.mav.command_long_send.assert_not_called()
        args = self.mav.set_position_target_local_ned_send.call_args.args
        self.assertEqual(args[8:11], (0, 0, 0))
        self.now += 5.1
        self.seed(ground=True, armed=False)
        self.b.tick()
        self.assertIsNone(self.b.bench)
        self.assertEqual(self.mav.set_position_target_local_ned_send.call_args.args[-1], 0)
