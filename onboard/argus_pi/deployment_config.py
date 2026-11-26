# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Shared deployment settings; no device, socket, or flight actions."""

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import tempfile

ROOT = Path(__file__).resolve().parent
ENV_KEYS = {
    "fc_state": "ARGUS_FOLLOW_B_FC_STATE_ENDPOINT",
    "mode_rpc": "ARGUS_FOLLOW_B_ONBOARD_RPC_ENDPOINT",
    "control": "ARGUS_FOLLOW_B_CONTROL_ENDPOINT",
    "operator": "ARGUS_FOLLOW_B_OPERATOR_ENDPOINT",
    # 2026-11-25 batch 4: 팀 vision 6374 (argus-onboard 가 bind). 예전 preview_input (5555),
    # preview_output (5558) 와 argus_preview_bus 는 없앴다
    "vision": "ARGUS_VISION_ENDPOINT",
    "tracks": "ARGUS_TRACKS_ENDPOINT",
}
# 예전 이름이 환경에 남아 있으면 받지 않는다 (뜻이 바뀐 값을 조용히 무시하지 않게)
RENAMED_ENV = {
    "ARGUS_BUS_INPUT_ENDPOINT": "ARGUS_VISION_ENDPOINT",
    "ARGUS_PREVIEW_ENDPOINT": "ARGUS_VISION_ENDPOINT",
}
# 2026-11-25 batch 4 리뷰: 예전 관측 포트 (5555 broker 입력, 5556, 5558 broker 출력). 이 값이
# ARGUS_OBSERVATIONS_ENDPOINT 에 남으면 추적기가 아무도 bind 하지 않는 곳을 기다리므로 받지 않는다
RETIRED_OBSERVATION_PORTS = (5555, 5556, 5558)


def _v21(data):
    profile = dict(
        schema="argus.follow.v2_1.v1",
        enabled=False,
        box_motion_guard=False,
        cv_gap=False,
        cv_visibility=False,
        vertical_center=False,
        vertical_center_weight=0.5,
        cv_gap_max_s=3.0,
        receipt_retry_ms=0,
        frame_diagnostics=False,
    )
    path = data["paths"].get("v2_1")
    if path:
        values = json.loads(Path(path).read_text(encoding="utf8"))
        if not isinstance(values, dict) or set(values) - set(profile):
            raise ValueError("Unknown V2.1 profile fields")
        profile.update(values)
    if profile["schema"] != "argus.follow.v2_1.v1":
        raise ValueError("Invalid V2.1 feature schema")
    for key in (
        "enabled",
        "box_motion_guard",
        "cv_gap",
        "cv_visibility",
        "vertical_center",
        "frame_diagnostics",
    ):
        if type(profile[key]) is not bool:
            raise ValueError("V2.1 feature must be boolean: " + key)
    if (
        type(profile["receipt_retry_ms"]) is not int
        or not 0 <= profile["receipt_retry_ms"] <= 50
        or type(profile["vertical_center_weight"]) not in (int, float)
        or profile["vertical_center_weight"] != 0.5
        or type(profile["cv_gap_max_s"]) not in (int, float)
        or profile["cv_gap_max_s"] != 3.0
    ):
        raise ValueError("V2.1 numeric contract differs from the bounded 3s/.5 profile")
    return profile


# 정비 선택값 -> (추종 설정 앞부분, 측정 설정, 카메라 보정; None이면 기본 45°).
# 30° 모드는 카메라 전용(FC 없는) 고정 보정도 30°판(CAMERA_ONLY_M30)을 쓴다.
OUTDOOR_PROFILES = {
    "predictive": ("config/follow/outdoor-", "config/tracking/measurement-outdoor.json", None),
    "predictive_low": (
        "config/follow/outdoor-low-",
        "config/tracking/measurement-outdoor.json",
        None,
    ),
    "predictive_m30": (
        "config/follow/outdoor-m30-",
        "config/tracking/measurement-outdoor-m30.json",
        "config/device/camera-mount30.json",
    ),
    "predictive_low_m30": (
        "config/follow/outdoor-low-m30-",
        "config/tracking/measurement-outdoor-m30.json",
        "config/device/camera-mount30.json",
    ),
}


# 2026-11-18 실내 두 갈래: 기본(runtime.json의 moving/yaw)과 데모 모드(회전 추종만, 고도 1.5 m).
# 데모 모드는 이동 칸과 회전 칸 모두 같은 demo-yaw.config를 쓴다. 측정/카메라는 실내 기본값.
# 값: (추종 설정, 측정 설정, 카메라 보정); None이면 runtime.json 실내 기본 경로 그대로.
INDOOR_PROFILES = {
    "predictive": None,
    "demo": (
        "config/follow/demo-yaw.config",
        "config/tracking/measurement.json",
        "config/device/camera.json",
    ),
    # 2026-11-19 데모2: yaw 고정 이동 추종. 네 번째 값은 회전 칸(없으면 이동 칸과 같은 파일).
    "demo2": (
        "config/follow/demo2-moving.config",
        "config/tracking/measurement.json",
        "config/device/camera.json",
        "config/follow/demo2-yaw.config",
    ),
}
CONTROLLER_NAMES = tuple(sorted(set(INDOOR_PROFILES) | set(OUTDOOR_PROFILES)))


CAMERA_ONLY_DEFAULT = "config/ucmc-camera-estimate.txt"
CAMERA_ONLY_M30 = "config/ucmc-camera-estimate-m30.txt"


def load(root=ROOT, environ=None):
    root = Path(root).resolve()
    env = os.environ if environ is None else environ
    data = json.loads((root / "config/runtime.json").read_text(encoding="utf-8"))
    environment = env.get("ARGUS_FOLLOW_ENVIRONMENT", "indoor")
    if environment not in ("indoor", "outdoor"):
        raise ValueError("Unknown follow environment")
    data["flight_environment"] = environment
    data["paths"].setdefault("camera_only", CAMERA_ONLY_DEFAULT)
    controller = env.get("ARGUS_FOLLOW_CONTROLLER", "predictive")
    # 2026-11-12 실외: 45° 장착 4 m·3 m, 30° 장착(지평면 아래) 4 m·3 m
    # 2026-11-18 실내: predictive(기본) 또는 demo(데모 모드). demo는 실내에서만 허용.
    allowed = INDOOR_PROFILES if environment == "indoor" else OUTDOOR_PROFILES
    if controller not in allowed:
        raise ValueError("Unknown follow profile for the selected environment")
    data["follow_controller"] = controller
    if environment == "indoor" and INDOOR_PROFILES[controller]:
        follow, measurement, camera, *yaw = INDOOR_PROFILES[controller]
        data["paths"].update(
            moving=follow, yaw=yaw[0] if yaw else follow, measurement=measurement, camera=camera
        )
    if environment == "outdoor":
        prefix, measurement, camera = OUTDOOR_PROFILES[controller]
        data["paths"].update(
            moving=prefix + "predictive-moving.config",
            yaw=prefix + "yaw.config",
            measurement=measurement,
        )
        if camera:
            data["paths"]["camera"] = camera
            data["paths"]["camera_only"] = CAMERA_ONLY_M30
    device = json.loads((root / "config/device/device.json").read_text(encoding="utf-8"))
    uid = device["expected_fc_uid"]
    if not isinstance(uid, str) or not re.fullmatch(r"[0-9a-fA-F]{36}", uid):
        raise ValueError("Invalid device expected_fc_uid; no implicit identity fallback")
    for key in ("expected_fc_system_id", "expected_fc_component_id"):
        if type(device[key]) is not int or not 1 <= device[key] <= 255:
            raise ValueError("Invalid device " + key)
    if not re.fullmatch(r"/dev/(?:tty(?:AMA|USB|ACM|S)[0-9]+|serial[01])", device["uart_device"]):
        raise ValueError("Invalid UART device")
    if type(device["uart_baud"]) is not int or device["uart_baud"] <= 0:
        raise ValueError("Invalid UART baud")
    for key in ("router_gateway_port", "router_legacy_port"):
        if type(device[key]) is not int or not 1024 <= device[key] <= 65535:
            raise ValueError("Invalid router port")
    if device["router_gateway_port"] == device["router_legacy_port"]:
        raise ValueError("Router port conflict")
    for variable, replacement in RENAMED_ENV.items():
        if variable in env:
            raise ValueError("Renamed endpoint variable: " + variable + " -> " + replacement)
    for key, variable in ENV_KEYS.items():
        value = env.get(variable, data["endpoints"][key])
        if not re.fullmatch(r"tcp://127\.0\.0\.1:[0-9]+", value):
            raise ValueError("Endpoint must be explicit loopback TCP: " + key)
        if not 1024 <= int(value.rsplit(":", 1)[1]) <= 65535:
            raise ValueError("Invalid endpoint port: " + key)
        data["endpoints"][key] = value
    if len(set(data["endpoints"].values())) != len(data["endpoints"]):
        raise ValueError("Endpoint ownership conflict")
    observation = env.get("ARGUS_OBSERVATIONS_ENDPOINT", data["endpoints"]["vision"])
    if (
        not re.fullmatch(r"tcp://127\.0\.0\.1:[0-9]+", observation)
        or not 1024 <= int(observation.rsplit(":", 1)[1]) <= 65535
    ):
        raise ValueError("Invalid observation endpoint")
    if int(observation.rsplit(":", 1)[1]) in RETIRED_OBSERVATION_PORTS:
        raise ValueError(
            "Retired endpoint value: ARGUS_OBSERVATIONS_ENDPOINT -> ARGUS_VISION_ENDPOINT"
        )
    data["observation_endpoint"] = observation
    for name, relative in data["paths"].items():
        path = (root / relative).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError("Missing/bad deployment path: " + name)
        data["paths"][name] = str(path)
    camera_override = env.get("ARGUS_FOLLOW_B_CAMERA_PROFILE")
    if camera_override:
        data["paths"]["camera"] = str(Path(camera_override).resolve(strict=True))
    camera = json.loads(Path(data["paths"]["camera"]).read_text(encoding="utf-8"))
    if camera["expected_uid"] != uid:
        raise ValueError("Camera calibration/device UID mismatch; recalibration required")
    data["v2_1"] = _v21(data)
    data["device"] = device
    data["root"] = str(root)
    modern_bin = env.get("ARGUS_BIN_DIR")
    legacy_bin = env.get("ARGUS_FOLLOW_B_BUILD_DIR")
    if modern_bin and legacy_bin and Path(modern_bin).resolve() != Path(legacy_bin).resolve():
        raise ValueError("Conflicting runtime binary directories")
    data["binary_dir"] = str(Path(modern_bin or legacy_bin or root / "bin").resolve())
    return data


def _atomic(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file() and path.read_text(encoding="utf-8") == content:
        return
    fd, name = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def materialize(data):
    """Render transport/identity only. Preserve every control/safety parameter."""
    # Per-settings output prevents an alternate connection from overwriting another's profile.
    import hashlib

    digest = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()[:16]
    output = Path(data["root"]) / "run/config" / digest
    for mode in ("moving", "yaw"):
        lines = Path(data["paths"][mode]).read_text(encoding="utf-8").splitlines()
        values = {
            "expected_fc_uid": data["device"]["expected_fc_uid"],
            "expected_fc_system_id": data["device"]["expected_fc_system_id"],
            "expected_fc_component_id": data["device"]["expected_fc_component_id"],
            "tracks_sub_endpoint": data["endpoints"]["tracks"],
            "tracks_pub_endpoint": data["endpoints"]["tracks"],
        }
        lines = [line for line in lines if line.partition("=")[0] not in values]
        content = "\n".join(lines + [str(k) + "=" + str(v) for k, v in values.items()]) + "\n"
        path = output / (mode + ".config")
        _atomic(path, content)
        data["paths"][mode] = str(path)
    device = data["device"]
    if any(device[key] in (14550, 14561) for key in ("router_gateway_port", "router_legacy_port")):
        raise ValueError("Configured router port conflicts with the common GCS bridge")
    router = (
        "[General]\nTcpServerPort = 0\nReportStats = false\n\n"
        "[UartEndpoint fc]\nDevice = {uart_device}\nBaud = {uart_baud}\nFlowControl = false\n\n"
        "[UdpEndpoint gui]\nMode = Server\nAddress = 127.0.0.1\nPort = {router_gateway_port}\n\n"
        "[UdpEndpoint yaw]\nMode = Normal\nAddress = 127.0.0.1\nPort = {router_legacy_port}\n\n"
        "[UdpEndpoint main_gcs_proxy]\nMode = Server\nAddress = 127.0.0.1\nPort = 14561\n"
    ).format(**device)
    _atomic(output / "mavlink-router.conf", router)
    data["router_config"] = str(output / "mavlink-router.conf")
    return data


def shell(data):
    values = {variable: data["endpoints"][key] for key, variable in ENV_KEYS.items()}
    values.update(
        ARGUS_OBSERVATIONS_ENDPOINT=data["observation_endpoint"],
        ARGUS_EXPECTED_FC_UID=data["device"]["expected_fc_uid"],
        ARGUS_MOVING_PROFILE=data["paths"]["moving"],
        ARGUS_YAW_PROFILE=data["paths"]["yaw"],
        ARGUS_CAMERA_PROFILE=data["paths"]["camera"],
        ARGUS_CAMERA_ONLY_CALIBRATION=data["paths"]["camera_only"],
        ARGUS_MEASUREMENT_PROFILE=data["paths"]["measurement"],
        ARGUS_UCMC_PROFILE=data["paths"]["ucmc"],
        ARGUS_BIN_DIR=data["binary_dir"],
        ARGUS_FOLLOW_B_BUILD_DIR=data["binary_dir"],
    )
    values.update(
        ARGUS_ROUTER_CONFIG=data.get("router_config", ""),
        ARGUS_UART_DEVICE=data["device"]["uart_device"],
    )
    values["ARGUS_FOLLOW_ENVIRONMENT"] = data["flight_environment"]
    values["ARGUS_FOLLOW_CONTROLLER"] = data["follow_controller"]
    profile = data["v2_1"]
    for field, variable in [
        ("enabled", "ARGUS_V21_ENABLED"),
        ("box_motion_guard", "ARGUS_V21_BOX_MOTION_GUARD"),
        ("cv_gap", "ARGUS_V21_CV_GAP"),
        ("cv_visibility", "ARGUS_V21_CV_VISIBILITY"),
        ("vertical_center", "ARGUS_V21_VERTICAL_CENTER"),
    ]:
        values[variable] = "1" if profile["enabled"] and profile[field] else "0"
    # Preserve the existing shared profile schema used by a running gateway.
    # V2.3's four tested options follow the already enabled CV/visibility flags.
    for variable, active in [
        ("ARGUS_V23_CV_RAW_MOTION", profile["cv_gap"]),
        ("ARGUS_V23_CV_MOVING_ONLY", profile["cv_gap"]),
        ("ARGUS_V23_CV_LATERAL", profile["cv_visibility"]),
        ("ARGUS_V23_CV_TILT_MODEL", profile["cv_visibility"]),
    ]:
        values[variable] = "1" if profile["enabled"] and active else "0"
    values["ARGUS_V21_RECEIPT_RETRY_MS"] = profile["receipt_retry_ms"] if profile["enabled"] else 0
    values["ARGUS_V21_FRAME_DIAG_PATH"] = (
        str(Path(data["root"]) / "run/follow_b/frame-diagnostic.jsonl")
        if profile["enabled"] and profile["frame_diagnostics"]
        else ""
    )
    return "\n".join(
        "export " + key + "=" + shlex.quote(str(value)) for key, value in values.items()
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shell", action="store_true")
    parser.add_argument("--materialize", action="store_true")
    parser.add_argument("--environment", choices=("indoor", "outdoor"))
    parser.add_argument("--controller", choices=CONTROLLER_NAMES)
    args = parser.parse_args()
    environment = dict(os.environ)
    if args.environment:
        environment["ARGUS_FOLLOW_ENVIRONMENT"] = args.environment
    if args.controller:
        environment["ARGUS_FOLLOW_CONTROLLER"] = args.controller
    settings = load(environ=environment)
    if args.materialize:
        settings = materialize(settings)
    print(shell(settings) if args.shell else json.dumps(settings, indent=2))
