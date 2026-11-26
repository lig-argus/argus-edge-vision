# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Real SDK-free tracker/controller against synthetic loopback onboard. No devices."""

import argparse
import json
import math
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import uuid

import zmq
import sys

# 2026-11-25 batch 2: tracking/tracks, tracking/status are team [name][protobuf] (6375).
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests/protobuf"))
import team_tracks_wire  # noqa: E402
import team_vision_wire  # noqa: E402  2026-11-25 batch 4: 6374 protobuf
import team_fc_wire  # noqa: E402  2026-11-25 batch 6a: 6370 protobuf
import team_control_wire  # noqa: E402  2026-11-25 batch 6b: 6372 protobuf
import team_mode_rpc_wire  # noqa: E402  2026-11-25 batch 7: 6371 protobuf
import team_follow_command_wire  # noqa: E402  2026-11-25 batch 8: 6373 protobuf


def endpoint():
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        return f"tcp://127.0.0.1:{reservation.getsockname()[1]}"


class Onboard:
    """Every fixture socket belongs to this one actor thread."""

    def __init__(self, uid):
        self.uid = uid
        self.telemetry, self.rpc, self.control, self.pixels, self.tracks = [
            endpoint() for _ in range(5)
        ]
        self.ready = threading.Event()
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.rc = False
        self.silent = False
        self.emit_detections = True
        self.offboard = False
        self.owner = False
        self.switch_at = None
        self.epoch = 7
        self.requests = []
        self.setpoints = []
        self.track_frames = []
        self.statuses = []
        self.mode_requests = []
        self.mode_events = []
        self.mode_replies = []
        self.rpc_errors = []  # 2026-11-25 batch 7: 6371 요청 해석 실패 (빈 본문으로 답함)
        self.error = None
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        assert self.ready.wait(2)

    def run(self):
        context = zmq.Context()
        tele = context.socket(zmq.PUB)
        tele.bind(self.telemetry)
        fc_wire = team_fc_wire.FcWire()  # 6370 팀 seq (토픽별 발행 번호)
        pixels = context.socket(zmq.PUB)
        pixels.bind(self.pixels)
        rpc = context.socket(zmq.REP)
        rpc.bind(self.rpc)
        output = context.socket(zmq.SUB)
        output.connect(self.control)
        output.subscribe(b"")
        tracks = context.socket(zmq.SUB)
        tracks.connect(self.tracks)
        tracks.subscribe(b"tracking/tracks")
        sockets = [tele, pixels, rpc, output, tracks]
        poller = zmq.Poller()
        for item in (rpc, output, tracks):
            poller.register(item, zmq.POLLIN)
        self.ready.set()
        frame_id = 0
        next_frame = 0
        pending = {}
        switch_request = None
        event_seq = 0
        reply_seq = 0

        def mode_event(request):
            nonlocal event_seq
            event_seq += 1
            event = dict(
                schema="argus.fc.mode_event.v1",
                source="argus-onboard",
                seq=event_seq,
                uid=self.uid,
                onboard_session="synthetic-onboard",
                session_id="synthetic-onboard",
                fc_epoch=self.epoch,
                clock_domain="host_monotonic",
                received_monotonic_ns=time.monotonic_ns(),
                requester_session_id=request["client_session"],
                command_id=request["request_id"],
                revision=request["revision"],
                command_epoch=request["command_epoch"],
                event="MODE_COMPLETED",
                actual_mode="OFFBOARD" if self.offboard else "HOLD",
                reason="actual_mode_confirmed",
                # 2026-11-25 batch 6a: protobuf 에서는 owner_token 이 늘 있다 (게이트웨이는 지금 토큰을 싣는다)
                owner_token="synthetic-owner",
            )
            self.mode_events.append(event)
            tele.send_multipart(fc_wire.frame(b"fc/mode_event", event))

        try:
            while not self.stop.is_set():
                now = time.monotonic()
                completed_request = None
                with self.lock:
                    if self.switch_at and now >= self.switch_at:
                        self.offboard = True
                        self.switch_at = None
                        completed_request, switch_request = switch_request, None
                    rc, silent, offboard, epoch = self.rc, self.silent, self.offboard, self.epoch
                if completed_request:
                    mode_event(completed_request)
                if now >= next_frame:
                    next_frame = now + 1 / 30
                    stamp = time.monotonic_ns()
                    if not silent:
                        common = dict(
                            uid=self.uid,
                            fc_epoch=epoch,
                            onboard_session="synthetic-onboard",
                            session_id="synthetic-onboard",
                            source="argus-onboard",
                            seq=frame_id + 1,
                            clock_domain="host_monotonic",
                            received_monotonic_ns=stamp,
                        )
                        attitude = dict(
                            common,
                            schema="argus.fc.attitude.v1",
                            roll_rad=0.0,
                            pitch_rad=0.0,
                            yaw_rad=0.0,
                        )
                        state = dict(
                            common,
                            schema="argus.fc.state.v1",
                            connected=True,
                            armed=True,
                            in_air=True,
                            offboard=offboard,
                            hold=not offboard,
                            position=False,
                            rc_override=rc,
                            ground_known=True,
                            landed=False,
                            local_ok=True,
                            flow_valid=True,
                            flow_quality=100,
                            north_m=0.0,
                            east_m=0.0,
                            down_m=-1.5,
                            vn_m_s=0.0,
                            ve_m_s=0.0,
                            vd_m_s=0.0,
                            distance_cm=150,
                            min_distance_cm=10,
                            max_distance_cm=1000,
                            distance_orientation=25,
                            position_monotonic_ns=stamp,
                            distance_monotonic_ns=stamp,
                        )
                        for topic, body in ((b"fc/attitude", attitude), (b"fc/state", state)):
                            tele.send_multipart(fc_wire.frame(topic, body))
                    frame_id += 1
                    observation = dict(
                        schema="argus.perception.observations.v2",
                        source="camera-synthetic",
                        frame_id=frame_id,
                        clock_domain="host_monotonic",
                        coordinate_frame="camera_pixel",
                        captured_monotonic_ns=stamp,
                        received_monotonic_ns=stamp,
                        image_width=640,
                        image_height=480,
                        detections=[
                            dict(
                                class_id=2,
                                label="car",
                                confidence=0.95,
                                x1=350.0,
                                y1=220.0,
                                x2=430.0,
                                y2=330.0,
                            )
                        ],
                    )
                    if not self.emit_detections:
                        observation["detections"] = []
                    pixels.send_multipart(team_vision_wire.observations_frame(observation))
                for item in dict(poller.poll(2)):
                    if item is rpc:
                        parts = rpc.recv_multipart()
                        # 2026-11-25 batch 7: 6371 은 팀 [control/mode_request][protobuf] 하나다.
                        # 모드 요청은 값의 schema 로 구분한다 (게이트웨이와 같다)
                        try:
                            request, mode_wire = team_mode_rpc_wire.receive_request(parts)
                        except ValueError as error:
                            self.rpc_errors.append(str(error))
                            rpc.send_multipart(team_mode_rpc_wire.empty_reply())
                            continue
                        if mode_wire:
                            self.mode_requests.append(request)
                        with self.lock:
                            self.requests.append(request)
                            action = request["action"]
                            response = {
                                key: request[key]
                                for key in (
                                    "schema",
                                    "request_id",
                                    "client_session",
                                    "onboard_session",
                                    "fc_epoch",
                                )
                            }
                            response["status"] = "accepted"
                            if action == "acquire":
                                self.owner = True
                                response.update(
                                    owner_token="synthetic-owner",
                                    capabilities=["context_v1", "absolute_heading_rad"],
                                )
                            elif action == "release":
                                self.owner = False
                            elif action == "start_offboard":
                                response["status"] = "pending"
                                pending[request["request_id"]] = time.monotonic() + 0.05
                                # Accepted ACK deliberately precedes actual telemetry mode.
                                self.switch_at = time.monotonic() + 0.4
                                if mode_wire:
                                    switch_request = request
                            elif action == "hold":
                                self.offboard = False
                                self.switch_at = None
                                response["status"] = "pending"
                                pending[request["request_id"]] = time.monotonic() + 0.03
                                if mode_wire:
                                    mode_event(request)
                            elif action == "request_status":
                                response["status"] = (
                                    "accepted"
                                    if time.monotonic()
                                    >= pending.get(request["original_request_id"], math.inf)
                                    else "pending"
                                )
                            # revoke cancels commands, never releases the independent owner.
                            elif action == "revoke":
                                for request_id in request.get("cancel_request_ids", []):
                                    pending.pop(request_id, None)
                        if mode_wire:
                            reply_seq += 1
                            response.update(
                                schema="argus.control.mode_result.v1",
                                result="ACCEPTED",
                                command_id=request["request_id"],
                                session_id="synthetic-onboard",
                                requester_session_id=request["client_session"],
                                uid=self.uid,
                                source="argus-onboard",
                                seq=reply_seq,
                                clock_domain="host_monotonic",
                                published_monotonic_ns=time.monotonic_ns(),
                            )
                            self.mode_replies.append(dict(response))
                        # 응답 첫 프레임은 늘 요청 이름이다 (팀 replier)
                        rpc.send_multipart(team_mode_rpc_wire.reply_frames(response))
                    elif item is output:
                        topic, packed = output.recv_multipart()
                        # 2026-11-25 batch 6b: 6372 는 팀 [이름][protobuf] (추종 제어 ZmqBus)
                        body = team_control_wire.decode(topic, packed)
                        with self.lock:
                            if topic == b"control/setpoint":
                                self.setpoints.append(
                                    dict(
                                        body,
                                        fixture_received_ns=time.monotonic_ns(),
                                        fixture_actual_offboard=self.offboard,
                                        fixture_rc=self.rc,
                                        fixture_owner=self.owner,
                                    )
                                )
                            elif topic == b"follow/status":
                                self.statuses.append(body)
                    else:
                        topic, packed = tracks.recv_multipart()
                        assert topic == team_tracks_wire.TRACKS, topic
                        body = team_tracks_wire.decode_tracks(packed)
                        with self.lock:
                            self.track_frames.append(body)
        except BaseException as error:
            self.error = repr(error)
        finally:
            for item in sockets:
                item.close(0)
            context.term()

    def close(self):
        self.stop.set()
        self.thread.join(3)
        assert not self.thread.is_alive()
        assert self.error is None, self.error


def wait(condition, name, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = condition()
        if value:
            return value
        time.sleep(0.02)
    raise AssertionError(name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--production-owner",
        action="store_true",
        help="Python FC gateway owner with mocked MAVLink",
    )
    parser.add_argument(
        "--icd",
        action="store_true",
        help="ICD numeric sensor, lifecycle, retry and scan deadline checks",
    )
    parser.add_argument(
        "--prediction-gap",
        action="store_true",
        help="Separate prediction reporting/observation deadlines",
    )
    parser.add_argument(
        "--recovery", action="store_true", help="Explicit STOP recovery, mock onboard only"
    )
    parser.add_argument(
        "--process-v2", action="store_true", help="PR26 common request/response contract"
    )
    parser.add_argument(
        "--shutdown-during-follow",
        action="store_true",
        help="Verify shutdown diagnostics during synthetic follow",
    )
    parser.add_argument(
        "--deferred-start",
        action="store_true",
        help="2026-11-17: START for a target missing in the latest frame waits instead of rejecting",
    )
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    uid = json.loads((root / "config/device/camera.json").read_text())["expected_uid"]
    if args.production_owner:
        from production_owner_fixture import ProductionOnboard

        onboard = ProductionOnboard(uid)
    else:
        onboard = Onboard(uid)
    context = zmq.Context()
    operator_endpoint = endpoint()
    sequence = 0
    controller_session = ""
    operator_requests = []

    def command(action, target=None, **extra):
        nonlocal sequence
        sequence += 1
        now = time.monotonic_ns()
        body = dict(
            schema="argus.follow.command.v1",
            action=action,
            command_id=uuid.uuid4().hex,
            expected_uid=uid,
            controller_session=controller_session,
            onboard_session="synthetic-onboard",
            fc_epoch=onboard.epoch,
            source="synthetic-operator",
            seq=sequence,
            issued_monotonic_ns=now,
            valid_until_monotonic_ns=now + 800000000,
        )
        if args.process_v2:
            body.update(
                schema="argus.follow.command.v2",
                session_id="synthetic-onboard",
                optional_future_field=1,
            )
        if target:
            body.update(target)
        body.update(extra)
        operator_requests.append(dict(body))
        if action == "STATUS" and not body["controller_session"]:
            body.pop("controller_session")
        client = context.socket(zmq.REQ)
        client.linger = 0
        client.connect(operator_endpoint)
        try:
            # 2026-11-25 batch 8: 6373 is team [follow/command][protobuf]. Keys outside the proto
            # (v2 optional_future_field) are dropped by the sender's to_proto.
            client.send_multipart(team_follow_command_wire.request_frames(body))
            if not client.poll(1500):
                return None
            reply = team_follow_command_wire.decode_reply(client.recv_multipart())
            if args.process_v2:
                assert reply["schema"] == "argus.follow.result.v2" and reply["result"] in (
                    "ACCEPTED",
                    "REJECTED",
                    "FAILED",
                ), reply
            return reply
        finally:
            client.close(0)

    processes = []
    report = dict(synthetic=True, devices_opened=False, pi_access=False, passed=False)
    with tempfile.TemporaryDirectory(prefix="argus-b-integration-") as folder:
        temporary = Path(folder)
        logs = []
        # 2026-11-25 batch 2 리뷰: 추적기 protobuf 변환이 키를 버리거나 실패하면 (v5 fc-ground) 실패
        tracker_wire_warnings = []
        try:
            profiles = []
            for name in ("moving", "yaw-only"):
                values = (
                    root
                    / (
                        "config/follow/yaw.config"
                        if name == "yaw-only"
                        else "config/follow/moving.config"
                    )
                ).read_text()
                overrides = dict(
                    tracks_sub_endpoint=onboard.tracks,
                    tracks_pub_endpoint=endpoint(),
                    expected_fc_uid=uid,
                    expected_fc_system_id="1",
                    expected_fc_component_id="1",
                    observations_sub_endpoint=onboard.pixels,
                    status_pub_endpoint=endpoint(),
                    command_rep_endpoint=endpoint(),
                    diagnostics_csv=str(temporary / (name + ".csv")),
                    prestream_setpoint_count="15" if args.production_owner else "3",
                    prestream_interval_ms="100" if args.production_owner else "20",
                )
                lines = [
                    key + "=" + overrides.get(key, value)
                    if "=" in line and not line.startswith("#")
                    else line
                    for line in values.splitlines()
                    for key, _, value in [line.partition("=")]
                ]
                # Deployment templates omit values supplied by the runtime loader.
                # Inject only this isolated mock's identity/endpoints into its temp file.
                present = {
                    line.partition("=")[0]
                    for line in lines
                    if "=" in line and not line.startswith("#")
                }
                lines += [
                    key + "=" + value for key, value in overrides.items() if key not in present
                ]
                path = temporary / (name + ".config")
                path.write_text("\n".join(lines) + "\n")
                profiles.append(path)
            identity = temporary / "generation"
            subprocess.run(
                [
                    str(args.build / "argus_follow_tracker"),
                    "--id-state-file",
                    str(identity),
                    "--initialize-id-state",
                ],
                check=True,
                capture_output=True,
            )
            commands = [
                [
                    str(args.build / "argus_follow_tracker"),
                    "--geometry",
                    "fc-ground",
                    "--calibration",
                    str(root / "config/device/camera.json"),
                    "--measurement",
                    str(root / "config/tracking/measurement.json"),
                    "--ucmc-profile",
                    str(root / "config/tracking/ucmc.json"),
                    "--expected-uid",
                    uid,
                    "--input",
                    onboard.pixels,
                    "--output",
                    onboard.tracks,
                    "--fc-state-endpoint",
                    onboard.telemetry,
                    "--id-state-file",
                    str(identity),
                ],
                [
                    str(args.build / "argus_follow_controller"),
                    "--moving-profile",
                    str(profiles[0]),
                    "--yaw-profile",
                    str(profiles[1]),
                    "--mode",
                    "yaw",
                    "--operator-endpoint",
                    operator_endpoint,
                    "--fc-state-endpoint",
                    onboard.telemetry,
                    "--onboard-rpc-endpoint",
                    onboard.rpc,
                    "--control-pub-endpoint",
                    onboard.control,
                ],
            ]
            if args.icd:
                commands[1] += ["--sensor-source", "1=camera-synthetic"]
            for index, argv in enumerate(commands):
                log = (temporary / f"process-{index}.log").open("w+")
                logs.append(log)
                processes.append(subprocess.Popen(argv, stdout=log, stderr=log, cwd=temporary))

            def candidates():
                status = command("STATUS")
                return (
                    status
                    if status and status.get("accepted") and status.get("candidates")
                    else None
                )

            status = wait(candidates, "native tracker candidates/controller status", 10)
            assert status["onboard_session"] == "synthetic-onboard", status
            assert status["fc_epoch"] == onboard.epoch, status
            assert status["clock_domain"] == "host_monotonic", status
            controller_session = status["controller_session"]
            target = status["candidates"][0]
            if args.icd:
                target = {key: value for key, value in target.items() if key != "label"}
                target["sensor"] = 1
            assert (
                command("START", target, controller_session="old-controller-session")["accepted"]
                is False
            )
            if args.deferred_start:
                # 2026-11-17 사용자 지시: 클릭한 대상이 누른 순간 프레임에 없어도 거부하지 않는다.
                # 탐지를 끈 동안 같은 대상이 후보에 남고, START는 진행 중으로 기다렸다가
                # 탐지가 돌아오면 실제 Offboard 추종으로 이어진다.
                onboard.emit_detections = False
                time.sleep(0.4)
                missing = command("STATUS")
                assert any(c["track_id"] == target["track_id"] for c in missing["candidates"]), (
                    missing
                )
                started = command("START", target)
                assert started["accepted"] and started["pending"], started
                time.sleep(0.5)
                waiting = command("STATUS")
                assert not waiting.get("active"), waiting
                assert not any(r["action"] == "start_offboard" for r in onboard.requests)
                onboard.emit_detections = True
                report["deferred_start_waited"] = True
            else:
                started = command("START", target)
            assert started["accepted"], started
            assert started["outcome"] == (
                "IN_PROGRESS" if started.get("pending") else "SUCCEEDED"
            ), started
            assert started["request_accepted"] is True
            if args.icd:
                assert started["pending"] and not started["execution_complete"], started
            wait(
                lambda: any(r["action"] == "start_offboard" for r in onboard.requests),
                "start mode RPC",
                8,
            )
            wait(
                lambda: (value := command("STATUS")) and value.get("active"),
                "actual Offboard becomes active",
                8,
            )

            def final_result(command_id):
                value = command("STATUS") or {}
                return next(
                    (
                        row
                        for row in value.get("command_results", [])
                        if row["command_id"] == command_id and not row["pending"]
                    ),
                    None,
                )

            if args.icd:
                assert (
                    wait(lambda: final_result(started["command_id"]), "START final result")[
                        "result"
                    ]
                    == "accepted"
                )
                before = sum(row["action"] == "start_offboard" for row in onboard.requests)
                repeated = command("START", target, command_id=started["command_id"])
                assert (
                    repeated["accepted"]
                    and repeated["execution_complete"]
                    and not repeated["pending"]
                ), repeated
                assert sum(row["action"] == "start_offboard" for row in onboard.requests) == before
            wait(
                lambda: any(abs(row.get("yaw_rate_rad_s", 0)) > 1e-8 for row in onboard.setpoints),
                "nonzero native yaw output",
                8,
            )
            assert all(
                row["fixture_actual_offboard"]
                for row in onboard.setpoints
                if any(
                    abs(row.get(key, 0)) > 1e-8
                    for key in ("vx_m_s", "vy_m_s", "vz_m_s", "yaw_rate_rad_s")
                )
            )
            denied_while_active = command("START", target)
            assert (
                not denied_while_active["accepted"] and denied_while_active["outcome"] == "REJECTED"
            ), denied_while_active
            assert (command("STATUS") or {}).get("active"), (
                "rejected command changed active following"
            )
            if args.recovery:
                assert not args.production_owner
                onboard.rc = True
                wait(
                    lambda: (x := command("STATUS")) and x.get("mission_state") == "MANUAL",
                    "manual latch",
                )
                with onboard.lock:
                    onboard.rc = False
                    onboard.offboard = False
                    onboard.switch_at = None
                time.sleep(0.15)
                assert command("STATUS")["mission_state"] == "MANUAL", (
                    "health recovery auto cleared latch"
                )
                starts_before = sum(x["action"] == "start_offboard" for x in onboard.requests)
                assert not command("START", target)["accepted"], "START bypassed manual latch"
                recovery_stop = command("STOP")
                assert recovery_stop["accepted"], recovery_stop
                wait(
                    lambda: (x := command("STATUS")) and x.get("mission_state") == "HOLD_WAIT",
                    "explicit STOP recovery Hold",
                )
                assert (
                    sum(x["action"] == "start_offboard" for x in onboard.requests) == starts_before
                )
                again = command("STOP", command_id=recovery_stop["command_id"])
                assert again["accepted"], again
                fresh = wait(candidates, "reselection after recovery")["candidates"][0]
                assert command("START", fresh)["accepted"]
                wait(
                    lambda: (x := command("STATUS")) and x.get("active"),
                    "new explicit START after recovery",
                )
                report["explicit_stop_recovery_verified"] = True
            if args.prediction_gap:
                onboard.emit_detections = False
                predicted = wait(
                    lambda: (x := command("STATUS")) and x.get("target_state") == "PREDICTED" and x,
                    "PREDICTED during valid retention",
                )
                deadline = predicted["prediction_valid_until_monotonic_ns"]
                assert (
                    deadline > time.monotonic_ns()
                    and predicted["observation_valid_until_monotonic_ns"] is None
                ), predicted
                time.sleep(0.15)
                with onboard.lock:
                    zero_begin = len(onboard.setpoints)
                time.sleep(0.2)
                again = command("STATUS")
                assert (
                    again["target_state"] == "PREDICTED"
                    and again["prediction_valid_until_monotonic_ns"] == deadline
                ), again
                with onboard.lock:
                    paused = list(onboard.setpoints[zero_begin:])
                assert paused and all(
                    abs(x.get(k, 0)) < 1e-9
                    for x in paused
                    for k in ("vx_m_s", "vy_m_s", "yaw_rate_rad_s")
                ), paused
                assert any(
                    x["status"].get("target_state") == "PREDICTED" for x in onboard.statuses
                ), "PUB omitted predicted state"
                onboard.emit_detections = True
                wait(
                    lambda: (x := command("STATUS")) and x.get("target_state") == "CONFIRMED",
                    "real observations confirmed again",
                )
                report["prediction_reporting_gap_verified"] = True
            stopped = command("STOP")
            assert stopped["accepted"], stopped
            wait(
                lambda: (
                    (value := command("STATUS"))
                    and not value.get("active")
                    and value.get("actual_hold")
                    and value.get("controls", {}).get("mode")
                ),
                "STOP and actual Hold completed",
                6,
            )
            if args.icd:
                assert (
                    wait(lambda: final_result(stopped["command_id"]), "STOP final result")["result"]
                    == "accepted"
                )
                assert not command("START", target, command_id=started["command_id"])["accepted"]
                if args.deferred_start:
                    onboard.emit_detections = False
                    time.sleep(0.4)
                    expiring = command("START", target)
                    assert expiring["accepted"] and expiring["pending"], expiring
                    expired = wait(
                        lambda: final_result(expiring["command_id"]),
                        "deferred START wait limit",
                        14,
                    )
                    assert expired["result"] == "failed", expired
                    assert expired["reason"] == "selected_target_not_detected_within_wait", expired
                    onboard.emit_detections = True
                    time.sleep(0.3)
                    report["deferred_start_expired"] = True
            assert command("SET_MODE", mode="moving")["accepted"]
            moving_status = wait(candidates, "fresh candidates after mode change")
            moving_target = moving_status["candidates"][0]
            with onboard.lock:
                moving_begin = len(onboard.setpoints)
            assert command("START", moving_target)["accepted"]
            wait(
                lambda: (value := command("STATUS")) and value.get("active"),
                "moving actual Offboard",
                8,
            )
            if args.shutdown_during_follow:
                shutdown_begin = time.monotonic_ns()
                processes[1].send_signal(signal.SIGTERM)
                try:
                    processes[1].wait(8)
                except subprocess.TimeoutExpired:
                    raise AssertionError("controller shutdown did not complete")
                path = temporary / "yaw-only.csv.decision.jsonl"
                rows = [json.loads(line) for line in path.read_text().splitlines() if line]
                assert any(
                    row.get("schema") == "argus.follow.decision.v1"
                    and row.get("phase") == "shutdown"
                    and row.get("tick_monotonic_ns", 0) >= shutdown_begin
                    for row in rows
                ), "shutdown decision not persisted"
                assert any(
                    row.get("schema") == "argus.follow.hold_diagnostic.v1"
                    and row.get("monotonic_ns", 0) >= shutdown_begin
                    for row in rows
                ), "shutdown HOLD diagnostic missing"
                report.update(
                    passed=True,
                    shutdown_decision_verified=True,
                    shutdown_hold_diagnostic_verified=True,
                )
                return
            wait(
                lambda: any(
                    abs(row.get("vx_m_s", 0)) > 1e-8 or abs(row.get("vy_m_s", 0)) > 1e-8
                    for row in onboard.setpoints[moving_begin:]
                ),
                "native moving translation",
                8,
            )
            # A selected real observation, the exact pose and candidate decisions must survive
            # the asynchronous bounded logger. They are estimates, not ground truth.
            if "predictive_follow=true" in profiles[0].read_text().lower():
                decision_path = temporary / "yaw-only.csv.decision.jsonl"

                def predictive_decision():
                    if not decision_path.exists():
                        return None
                    try:
                        rows = [
                            json.loads(line)
                            for line in decision_path.read_text().splitlines()
                            if line
                        ]
                    except (OSError, json.JSONDecodeError):
                        return None  # writer may be appending the final line
                    return next(
                        (
                            row
                            for row in reversed(rows)
                            if row.get("predictive_input_capture_failed") is False
                            and row.get("command_sent")
                            and row.get("mission") == "FOLLOW_ACTIVE"
                            and row.get("vertical_decision")
                            and row.get("target")
                        ),
                        None,
                    )

                trace = wait(predictive_decision, "predictive decision JSONL flush", 8)
                assert trace["schema"] == "argus.follow.decision.v1"
                assert (
                    trace["session_start_monotonic_ns"] > 0
                    and trace["session_start_unix_ns_unverified"] > 0
                )
                assert trace["clock_domain"] == "host_monotonic"
                frame, target, pose, camera, vertical = (
                    trace[k]
                    for k in (
                        "frame",
                        "target",
                        "vehicle_pose",
                        "camera_calibration",
                        "vertical_decision",
                    )
                )
                assert frame["sample_monotonic_ns"] == trace["command_sample_ns"] > 0
                assert frame["width"] > 0 and frame["height"] > 0 and frame["generation"] > 0
                assert (
                    target["track_id"] > 0
                    and target["real_detection"]
                    and target["state"] == "CONFIRMED"
                )
                x1, y1, x2, y2 = target["bbox_xyxy_px"]
                assert 0 <= x1 < x2 <= frame["width"] and 0 <= y1 < y2 <= frame["height"]
                assert len(target["ground_anchor_ne_cov_alignment"]) == 6
                assert target["estimated_range_valid"] and target["range_quality"] == "METRIC"
                assert frame["observation_pose_origin"] in (
                    "producer_ground_pose",
                    "aligned_pose_history",
                )
                assert (
                    trace["observation_pose"]["have_position"]
                    and pose["have_position"]
                    and pose["have_attitude"]
                )
                assert len(pose["ned_m"]) == len(pose["velocity_ned_m_s"]) == 3
                assert camera["fx"] > 0 and camera["fy"] > 0 and len(camera["camera_to_body"]) == 9
                assert trace["sent_command"]["frame"] == "body_frd"
                assert math.isclose(
                    trace["sent_command"]["vz_down_m_s"], vertical["final_down_m_s"], abs_tol=1e-9
                )
                assert math.isclose(
                    sum(
                        vertical[k]
                        for k in ("cost_height", "cost_visibility", "cost_boundary", "cost_other")
                    ),
                    vertical["winning_plan_cost_before_slew"],
                    rel_tol=1e-7,
                    abs_tol=1e-7,
                )
                assert 0 <= vertical["safety_flags"] <= 63 and vertical["slew_m_s2"] > 0
                assert vertical["candidate_score_basis"].startswith("minimum evaluated coarse-grid")
                candidates_present = [
                    vertical[k]
                    for k in ("candidate_up_score", "candidate_zero_score", "candidate_down_score")
                    if k in vertical
                ]
                assert candidates_present and all(
                    math.isfinite(value) for value in candidates_present
                )
                assert trace["configuration"]["range_m"] > 0 and "follow_goal" in trace
                report["predictive_decision_record_verified"] = True
            stopped = command("STOP")
            assert stopped["accepted"], stopped
            wait(
                lambda: (
                    (value := command("STATUS"))
                    and not value.get("active")
                    and value.get("actual_hold")
                    and value.get("controls", {}).get("mode")
                ),
                "moving STOP and actual Hold completed",
                6,
            )
            headings_before = sum(row["action"] == "heading" for row in onboard.requests)
            scan = command("SCAN_START", yaw_range_rad=1.0, yaw_rate_rad_s=0.2, timeout_s=5.0)
            assert scan["accepted"], scan
            wait(
                lambda: (
                    sum(row["action"] == "heading" for row in onboard.requests) > headings_before
                ),
                "absolute heading emitted by actual Scan path",
            )
            stopped = command("STOP")
            assert stopped["accepted"], stopped
            wait(
                lambda: (
                    (value := command("STATUS"))
                    and not value.get("heading_stop_pending")
                    and value.get("controls", {}).get("mode")
                ),
                "Scan STOP terminal heading acknowledged",
                6,
            )
            if args.icd:
                onboard.emit_detections = False
                time.sleep(0.35)
                limited = command(
                    "SCAN_START", yaw_range_rad=0.02, yaw_rate_rad_s=0.2, timeout_s=0.35
                )
                assert limited["accepted"] and limited["pending"], limited
                active_scan = command("STATUS")
                assert (
                    active_scan["scan_remaining_s"] is not None
                    and 0 <= active_scan["scan_remaining_s"] <= 0.35
                ), active_scan
                report["scan_status_example"] = active_scan
                completed = wait(
                    lambda: final_result(limited["command_id"]), "Scan time-limit final result"
                )
                assert (
                    completed["result"] == "accepted"
                    and completed["reason"] == "scan_time_limit_completed"
                ), completed
                state = command("STATUS")
                assert completed["outcome"] == "SUCCEEDED" and completed["request_accepted"]
                assert math.isclose(state["scan_config"]["yaw_rate_rad_s"], 0.2, abs_tol=1e-12)
                assert state["scan_config"]["timeout_s"] == 0.35
                assert (
                    state["mission_state"] == "HOLD_WAIT" and state["scan_state_name"] == "TimedOut"
                ), state
                assert state["scan_remaining_s"] is None and state["scan_yaw_done_rad"] is None, (
                    state
                )
                onboard.emit_detections = True
                time.sleep(0.1)
            with onboard.lock:
                actions = [row["action"] for row in onboard.requests]
                assert "revoke" in actions and "hold" in actions and "release" in actions, actions
                frames = list(onboard.track_frames)
            assert frames and all(row["source"] == "camera-synthetic" for row in frames)
            assert all(type(row["seq"]) is int for row in frames)
            assert onboard.mode_requests, "no PR26 multipart mode request"
            assert all(
                row["command_id"] == row["request_id"]
                and row["session_id"] == row["client_session"]
                for row in onboard.mode_requests
            )
            assert all(
                row["exposure_monotonic_ns"] is None and not row["exposure_time_valid"]
                for row in frames
            )
            onboard.rc = True
            time.sleep(0.15)
            rejected = command("START", moving_target)
            assert not rejected["accepted"], rejected
            assert rejected["outcome"] == "REJECTED" and not rejected["request_accepted"], rejected
            onboard.rc = False
            onboard.silent = True
            time.sleep(0.35)
            stale = command("START", moving_target)
            assert (stale is not None and not stale["accepted"]) or processes[
                1
            ].poll() is not None, stale
            report.update(
                passed=True,
                tracker_frames=len(frames),
                control_packets=len(onboard.setpoints),
                rpc_actions=actions,
                delayed_actual_mode_verified=True,
                stop_hold_release_verified=True,
                moving_translation_verified=True,
                scan_heading_and_terminal_stop_verified=True,
                stale_fail_closed_exit=processes[1].poll() is not None,
                stale_controller_nonce_rejected=True,
                rc_and_stale_blocked=True,
                original_frame_source_preserved=True,
                exposure_unverified_preserved=True,
            )
            report.update(
                mode_request_multipart=True,
                track_sequence_preserved=True,
                process_request_v2=args.process_v2,
            )
            report["follow_result_examples"] = [
                started,
                repeated if args.icd else started,
                denied_while_active,
                rejected,
            ]
            report["follow_status_examples"] = list(onboard.statuses)
            assert report["follow_status_examples"], "no status publication"
            for sample in report["follow_status_examples"]:
                cfg = sample["status"]["scan_config"]
                assert math.isclose(
                    cfg["yaw_rate_rad_s"], math.radians(cfg["rate_deg_s"]), abs_tol=1e-12
                )
                assert cfg["timeout_s"] == cfg["timeout_ms"] / 1000
                for row in sample["status"]["command_results"]:
                    assert row["outcome"] in ("IN_PROGRESS", "SUCCEEDED", "REJECTED", "FAILED")
                    assert row["request_accepted"] == (row["outcome"] != "REJECTED")
            report["wire_examples"] = dict(
                mode_request=onboard.mode_requests[0],
                mode_result=onboard.mode_replies[0] if onboard.mode_replies else None,
                mode_event=onboard.mode_events[0] if onboard.mode_events else None,
                tracking_tracks=frames[0],
                follow_result=started,
                follow_command=next(
                    row for row in operator_requests if row["command_id"] == started["command_id"]
                ),
            )
            if args.icd:
                report.update(
                    icd_numeric_sensor_label_resolution=True,
                    icd_command_final_results=True,
                    retry_no_duplicate_execution=True,
                    late_start_after_stop_rejected=True,
                    scan_timeout_normal_after_freeze=True,
                )
        finally:
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGTERM)
            for index, (process, log) in enumerate(zip(processes, logs)):
                try:
                    process.wait(5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                log.seek(0)
                report[f"process_{index}_returncode"] = process.returncode
                text = log.read()
                report[f"process_{index}_log"] = text[-12000:]
                # 2026-11-25 batch 6b: 추종 제어 (index 1) 도 팀 ZmqBus 로 6372 를 보낸다
                if index in (0, 1):
                    tracker_wire_warnings += [
                        line
                        for line in text.splitlines()
                        if "PB_UNKNOWN_FIELD" in line or "pi_codec_table:" in line
                    ]
                log.close()
            onboard.close()
            context.term()
            if hasattr(onboard, "production_diagnostics"):
                report["production_owner"] = onboard.production_diagnostics
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2))
        assert not tracker_wire_warnings, tracker_wire_warnings
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
