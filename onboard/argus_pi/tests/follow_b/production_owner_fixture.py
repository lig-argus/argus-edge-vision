# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Native process integration using production Python owner and mocked MAVLink."""

import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch
import msgpack
import zmq
from process_integration_test import Onboard
import team_tracks_wire  # tests/protobuf (process_integration_test 가 경로를 넣는다)
import team_vision_wire  # noqa: E402  2026-11-25 batch 4: 6374 protobuf
import team_control_wire  # noqa: E402  2026-11-25 batch 6b: 6372 protobuf

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "apps/real_fc"))
from tools.real_fc.vehicle import FlightLink, MODES
from tools.real_fc.config import INDOOR_PROFILE
from tools.real_fc.follow_b_owner import FollowBOwner


class ProductionOnboard(Onboard):
    def run(self):
        context = zmq.Context()
        pixels = context.socket(zmq.PUB)
        pixels.bind(self.pixels)
        tracks = context.socket(zmq.SUB)
        tracks.connect(self.tracks)
        tracks.subscribe(b"tracking/tracks")
        status = context.socket(zmq.SUB)
        status.connect(self.control)
        status.subscribe(b"follow/status")
        b = None
        try:
            with patch.object(FlightLink, "_run", return_value=None):
                v = FlightLink(self.uid)
            v.uid = self.uid
            v.pose_epoch = self.epoch
            v.params = dict(
                INDOOR_PROFILE, EKF2_OF_QMIN=1, COM_POS_FS_EPH=5.0, SENS_FLOW_MAXHGT=10.0
            )
            mav = Mock()
            v.link = SimpleNamespace(mav=mav, close=lambda: None)
            b = FollowBOwner(
                v,
                (self.telemetry, self.rpc, self.control),
                offboard_hover=getattr(self, "hover_enabled", False),
            )
            v.follow_b = b
            b.session = "synthetic-onboard"
            request = b.request

            def record_request(d):
                result = request(d)
                self.requests.append(dict(d, fixture_reply=result))
                return result

            b.request = record_request
            mode_request = b.mode_request

            def record_mode(d):
                self.mode_requests.append(d)
                result = mode_request(d)
                self.mode_replies.append(dict(result))
                return result

            b.mode_request = record_mode
            publish = b._publish

            def record_publish(topic, data):
                publish(topic, data)
                if topic == b"fc/mode_event":
                    self.mode_events.append(
                        dict(data, source="argus-onboard", seq=b.pub_seq, session_id=b.session)
                    )

            b._publish = record_publish
            self.vehicle = v
            self.owner = b
            self.hover_records = []
            rejected = []
            receive = b.receive_setpoint

            def record_receive(d):
                result = receive(d)
                if not result:
                    rejected.append(b.last_reason)
                return result

            b.receive_setpoint = record_receive
            self.fc_mode_commands = []

            def mode_command(*args):
                self.fc_mode_commands.append(args)
                if args[2] == 176:
                    self.switch_at = time.monotonic() + 0.03
                    self.switch_mode = "offboard" if args[5] == 6 else "hold"

            mav.command_long_send.side_effect = mode_command

            def packet(*args):
                self.setpoints.append(
                    dict(
                        vx_m_s=args[8],
                        vy_m_s=args[9],
                        vz_m_s=args[10],
                        yaw_rate_rad_s=args[-1],
                        fixture_actual_offboard=self.offboard,
                    )
                )

            mav.set_position_target_local_ned_send.side_effect = packet
            direct = getattr(self, "direct_handoff", False)
            if direct:
                self.offboard = True
                v.external = v.owned_flight = v.owned_mode_confirmed = True
                v._gcs_active = lambda: getattr(self, "gui_alive", True)
                v.command_owner.acquire()
                v.local_target = dict(
                    x=0.0,
                    y=0.0,
                    start_z=-1.5,
                    target_z=-1.5,
                    distance=0.0,
                    height=1.5,
                    yaw=0.0,
                    climb_at=time.monotonic() - 10,
                )
            admitted = False
            self.ready.set()
            frame_id = 0
            next_frame = 0
            while not self.stop.is_set():
                now = time.monotonic()
                if getattr(self, "gui_alive", True):
                    v.lease_at = now
                v.pose_epoch = self.epoch
                if self.switch_at and now >= self.switch_at:
                    self.offboard = self.switch_mode == "offboard"
                    self.switch_at = None
                    next_frame = now
                    b.ingest("COMMAND_ACK", dict(command=176, result=0))
                if self.rc and not b.rc_override:
                    b.ingest("STATUSTEXT", dict(text="Pilot took over using sticks"))
                if now >= next_frame:
                    next_frame = now + 1 / 30
                    stamp = time.monotonic_ns()
                    if not self.silent:
                        main, sub = MODES["offboard" if self.offboard else "hold"]
                        v.samples = dict(
                            HEARTBEAT=(
                                dict(
                                    base_mode=0 if getattr(self, "ground_test", False) else 128,
                                    custom_mode=main << 16 | sub << 24,
                                ),
                                now,
                            ),
                            EXTENDED_SYS_STATE=(
                                dict(landed_state=1 if getattr(self, "ground_test", False) else 2),
                                now,
                            ),
                            ESTIMATOR_STATUS=(dict(flags=79, pos_horiz_accuracy=0.1), now),
                            LOCAL_POSITION_NED=(
                                dict(x=0.0, y=0.0, z=-1.5, vx=0.0, vy=0.0, vz=0.0),
                                now,
                            ),
                            ATTITUDE=(dict(roll=0.0, pitch=0.0, yaw=0.0), now),
                            DISTANCE_SENSOR=(
                                dict(
                                    current_distance=150,
                                    min_distance=10,
                                    max_distance=1000,
                                    orientation=25,
                                ),
                                now,
                            ),
                            OPTICAL_FLOW_RAD=(dict(quality=100), now),
                        )
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
                if direct:
                    if not admitted and v.samples:
                        v.yaw_handoff()
                        admitted = True
                    if getattr(self, "new_handoff", False):
                        v.yaw_handoff()
                        self.new_handoff = False
                    v._control_tick(supervised=True)
                else:
                    b.tick()
                if b.hover_ready():
                    self.hover_records.append((now, v.local_target["target_z"]))
                while status.poll(0):
                    parts = status.recv_multipart()
                    if len(parts) == 2:
                        self.statuses.append(team_control_wire.decode(parts[0], parts[1]))
                while tracks.poll(0):
                    parts = tracks.recv_multipart()
                    if len(parts) == 2 and parts[0] == team_tracks_wire.TRACKS:
                        self.track_frames.append(team_tracks_wire.decode_tracks(parts[1]))
                time.sleep(0.002)
        except Exception as exc:
            self.error = repr(exc)
            self.ready.set()
        finally:
            if b:
                self.production_diagnostics = dict(
                    reason=b.last_reason,
                    rejected=rejected,
                    requests=self.requests,
                    mode_requests=self.mode_requests,
                    mode_replies=self.mode_replies,
                    mode_events=self.mode_events,
                    statuses=self.statuses,
                    headings=[
                        dict(stopping=r.get("stopping"), reply=r.get("fixture_reply"))
                        for r in self.requests
                        if r["action"] == "heading"
                    ],
                    setpoints=len(self.setpoints),
                    frames=len(self.track_frames),
                )
                b.close()
            pixels.close(0)
            tracks.close(0)
            status.close(0)
            context.term()
