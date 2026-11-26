# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Real tracker/manager/proxy with synthetic input and no physical FC or camera."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid
import zmq
from process_integration_test import Onboard, endpoint, wait

# 2026-11-25 batch 8: 6373 is team [follow/command][protobuf]; the forged START below uses the
# same pb_json codec as tools.real_fc.follow_proxy.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests/protobuf"))
import team_follow_command_wire  # noqa: E402

root, build = map(Path, sys.argv[1:3])
sys.path.insert(0, str(root / "apps/real_fc"))
from tools.real_fc import follow_proxy as proxy

uid = json.loads((root / "config/device/camera.json").read_text())["expected_uid"]
fake = Onboard(uid)
fake.silent = True
proxy.ENDPOINT = endpoint()
children = []
with tempfile.TemporaryDirectory(prefix="argus-observation-selection-") as folder:
    temp = Path(folder)
    logs = []
    try:
        profiles = []
        for mode in ("moving", "yaw"):
            data = dict(
                line.split("=", 1)
                for line in (root / f"config/follow/{mode}.config").read_text().splitlines()
                if "=" in line and not line.startswith("#")
            )
            data.update(
                expected_fc_uid=uid,
                expected_fc_system_id="1",
                expected_fc_component_id="1",
                tracks_sub_endpoint=fake.tracks,
                tracks_pub_endpoint=endpoint(),
                observations_sub_endpoint=fake.pixels,
                status_pub_endpoint=endpoint(),
                command_rep_endpoint=endpoint(),
                diagnostics_csv=str(temp / (mode + ".csv")),
            )
            path = temp / (mode + ".config")
            path.write_text("\n".join(k + "=" + v for k, v in data.items()) + "\n")
            profiles.append(path)
        ids = temp / "generation"
        subprocess.run(
            [
                str(build / "argus_follow_tracker"),
                "--id-state-file",
                str(ids),
                "--initialize-id-state",
            ],
            check=True,
            capture_output=True,
        )
        commands = [
            [
                str(build / "argus_follow_tracker"),
                "--geometry",
                "auto",
                "--calibration",
                str(root / "config/device/camera.json"),
                "--camera-only-calibration",
                str(root / "config/ucmc-camera-estimate.txt"),
                "--measurement",
                str(root / "config/tracking/measurement.json"),
                "--expected-uid",
                uid,
                "--ucmc-profile",
                str(root / "config/tracking/ucmc.json"),
                "--input",
                fake.pixels,
                "--output",
                fake.tracks,
                "--fc-state-endpoint",
                fake.telemetry,
                "--id-state-file",
                str(ids),
            ],
            [
                str(build / "argus_follow_controller"),
                "--moving-profile",
                str(profiles[0]),
                "--yaw-profile",
                str(profiles[1]),
                "--mode",
                "yaw",
                "--operator-endpoint",
                proxy.ENDPOINT,
                "--fc-state-endpoint",
                fake.telemetry,
                "--onboard-rpc-endpoint",
                fake.rpc,
                "--control-pub-endpoint",
                fake.control,
            ],
        ]
        for i, command in enumerate(commands):
            log = (temp / f"{i}.log").open("w+")
            logs.append(log)
            children.append(subprocess.Popen(command, stdout=log, stderr=log))

        def status():
            try:
                return proxy.forward_vmy(
                    dict(action="status", expected_uid=uid), uid, timeout_ms=200
                )
            except ValueError:
                return {}

        ready = wait(
            lambda: s if (s := status()).get("candidates") else None, "no-FC candidates", 8
        )
        assert ready["observation_ready"] and not ready["identity_ok"] and not ready["active"]
        candidate = ready["candidates"][0]
        selected = proxy.forward_vmy(dict(action="select", expected_uid=uid, **candidate), uid)
        assert selected["accepted"], selected
        assert status()["selected"] == {k: candidate[k] for k in ("generation", "track_id")}
        for action in ("start", "prepare_handoff", "search", "mode"):
            try:
                proxy.forward_vmy(
                    dict(action=action, expected_uid=uid, mode="yaw", **candidate), uid
                )
            except ValueError:
                pass
            else:
                raise AssertionError("FC-less flight action admitted " + action)
        context = zmq.Context()
        sock = context.socket(zmq.REQ)
        sock.linger = 0
        sock.connect(proxy.ENDPOINT)
        stamp = time.monotonic_ns()
        forged = dict(
            schema="argus.follow.command.v1",
            action="START",
            command_id=uuid.uuid4().hex,
            source="test",
            seq=1,
            expected_uid=uid,
            clock_domain="host_monotonic",
            issued_monotonic_ns=stamp,
            valid_until_monotonic_ns=stamp + 900000000,
            **candidate,
            **{k: ready[k] for k in ("controller_session", "onboard_session", "fc_epoch")},
        )
        sock.send_multipart(team_follow_command_wire.request_frames(forged))
        assert sock.poll(1500)
        forged_reply = team_follow_command_wire.decode_reply(sock.recv_multipart())
        assert not forged_reply["accepted"], forged_reply
        assert forged_reply["command_id"] == forged["command_id"], forged_reply
        sock.close()
        context.term()
        time.sleep(5.2)  # Remain alive beyond the former five-second FC bootstrap timeout.
        assert all(p.poll() is None for p in children)
        fake.silent = False
        recovered = wait(lambda: s if (s := status()).get("identity_ok") else None, "FC recovery")
        assert not recovered["active"]
        recovered = wait(
            lambda: s if (s := status()).get("input_contract_v5") and s.get("candidates") else None,
            "fresh FC candidate list",
        )
        assert recovered["selected"] is None and not recovered["active"]
        fake.silent = True
        lost = wait(
            lambda: (
                s
                if (s := status()).get("tracking_only")
                and not s.get("identity_ok")
                and s.get("candidates")
                else None
            ),
            "FC loss camera list",
        )
        assert proxy.forward_vmy(
            dict(action="select", expected_uid=uid, **lost["candidates"][0]), uid
        )["accepted"]
        assert not fake.requests and not fake.mode_requests and not fake.setpoints, (
            fake.requests,
            fake.mode_requests,
            fake.setpoints,
        )
        print(
            json.dumps(
                dict(
                    passed=True,
                    no_fc_candidates=True,
                    native_selection=True,
                    direct_start_rejected=True,
                    fc_recovery_no_auto_follow=True,
                    owner_requests=0,
                    mode_requests=0,
                    setpoints=0,
                )
            )
        )
    finally:
        for child in children:
            child.terminate()
            try:
                child.wait(5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        for log in logs:
            log.seek(0)
            print(log.read()[-3000:])
            log.close()
        fake.close()
