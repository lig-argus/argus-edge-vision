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
    parser.add_argument("--direct-mode", choices=["yaw", "moving"], default="yaw")
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--rgb-profile", action="store_true")
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
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    uid = json.loads((root / "config/device/camera.json").read_text())["expected_uid"]
    if args.production_owner:
        from production_owner_fixture import ProductionOnboard

        class DirectOnboard(ProductionOnboard):
            direct_handoff = True
            hover_enabled = True

        onboard = DirectOnboard(uid)
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
                    str(
                        root
                        / (
                            "config/device/camera-rgb.json"
                            if args.rgb_profile
                            else "config/device/camera.json"
                        )
                    ),
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
            if args.rgb_profile:
                commands[1] += ["--camera-profile", str(root / "config/device/camera-rgb.json")]
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
            assert command("PREPARE_HANDOFF")["accepted"]
            assert command("SET_MODE", mode=args.direct_mode)["accepted"]
            assert command("SELECT", target)["accepted"]
            before = len(onboard.setpoints)
            started = command("START", target)
            assert started["accepted"], started
            wait(
                lambda: (value := command("STATUS")) and value.get("active"),
                "direct Offboard active",
                8,
            )
            field = "yaw_rate_rad_s" if args.direct_mode == "yaw" else "vx_m_s"
            wait(
                lambda: any(abs(row[field]) > 1e-8 for row in onboard.setpoints[before:]),
                "real native guidance output",
                8,
            )
            assert not onboard.mode_requests, onboard.mode_requests
            assert not any(row["action"] in ("start_offboard", "hold") for row in onboard.requests)
            assert all(row["fixture_actual_offboard"] for row in onboard.setpoints)
            onboard.gui_alive = False
            wait(
                lambda: onboard.owner.hover_ready() and not onboard.owner.token,
                "GUI lease loss passive hover",
                7,
            )
            count = len(onboard.setpoints)
            time.sleep(1.2)
            assert len(onboard.setpoints) > count + 10, "hover stream stopped with GUI loss"
            assert not onboard.fc_mode_commands, "AUTO_HOLD/flight mode request sent"
            assert onboard.offboard
            onboard.gui_alive = True
            time.sleep(0.7)
            assert not command("STATUS")["active"], (
                "connection recovery automatically resumed tracking"
            )
            stopped = command("STOP")
            assert stopped["accepted"], stopped
            onboard.new_handoff = True
            wait(lambda: not onboard.new_handoff, "explicit new handoff")
            assert command("PREPARE_HANDOFF")["accepted"]
            assert command("SET_MODE", mode=args.direct_mode)["accepted"]
            target = wait(candidates, "fresh target")["candidates"][0]
            assert command("SELECT", target)["accepted"]
            assert command("START", target)["accepted"]
            wait(
                lambda: command("STATUS").get("active"),
                "explicit restart after connection recovery",
                5,
            )
            assert not onboard.fc_mode_commands
            report.update(
                passed=True,
                mode=args.direct_mode,
                gui_lease_loss_hover=True,
                hover_continued=True,
                no_automatic_resume=True,
                explicit_restart=True,
                no_auto_hold=True,
                packets=len(onboard.setpoints),
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
                report[f"process_{index}_log"] = log.read()[-12000:]
                log.close()
            onboard.close()
            context.term()
            if hasattr(onboard, "production_diagnostics"):
                report["production_owner"] = onboard.production_diagnostics
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
