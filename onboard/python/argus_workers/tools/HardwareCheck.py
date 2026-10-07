"""Static checks only. C++ validates profiles and owns camera I/O."""
import argparse
import importlib
import os
from pathlib import Path
import subprocess


def main():
    root = Path(__file__).resolve().parents[3]
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--profile", default="REAL_IR")
    p.add_argument("--profiles", default=str(root / "config/profiles.toml"))
    p.add_argument("--frame-source", default="auto")
    p.add_argument("--camera", default="/dev/video0")
    p.add_argument("--hef", required=True)
    p.add_argument("--labels", required=True)
    p.add_argument("--ir-capture", default="/home/user/thessen_raw14_viewer/build/i3_raw14_capture")
    p.add_argument("--ir-device", type=int, default=0)
    p.add_argument("--ir-timeout", type=int, default=15)
    args = p.parse_args()
    for name in ("numpy", "cv2", "zmq", "msgpack", "hailo_platform"):
        importlib.import_module(name)
    if not Path(args.hef).is_file() or not Path(args.labels).is_file():
        raise SystemExit("HEF/labels file missing")
    if not Path("/dev/hailo0").exists():
        raise SystemExit("Hailo device missing")
    config = subprocess.check_output([str(root / "build/bin/argus-onboard"), "--check-config",
        "--profiles", args.profiles, "--profile", args.profile, "--frame-source", args.frame_source,
        "--backend", "hailo", "--hef", args.hef, "--labels", args.labels], text=True)
    kind = next(token.split("=", 1)[1] for token in config.split() if token.startswith("frame_source="))
    if kind == "ir_sdk":
        if not os.access(args.ir_capture, os.X_OK):
            raise SystemExit("SDK capture executable missing")
    elif kind == "ir_v4l2":
        device = Path(f"/dev/video{args.camera}" if args.camera.isdecimal() else args.camera)
        if not device.exists():
            raise SystemExit(f"camera device missing: {device}")
    elif kind == "replay":
        if not Path(args.camera).is_file():
            raise SystemExit("replay file missing")
    else:
        raise SystemExit(f"frame source adapter not implemented: {kind}")
    from hailo_platform import HEF
    info = HEF(args.hef).get_input_vstream_infos()
    if len(info) != 1 or info[0].shape[2] != 3:
        raise SystemExit("HEF must have one three-channel input")
    print(config.strip())
    print("Static checks passed. No camera stream, Hailo inference, or PX4 command was started.")


if __name__ == "__main__":
    main()
