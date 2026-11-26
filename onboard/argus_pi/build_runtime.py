# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Unified build. ARM installs bin/; --host-check never installs or opens hardware."""

import argparse
import os
import hashlib
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parent
# 2026-11-25 batch 4: argus_preview_bus (5555 -> 5558 프록시) 는 없앴다. 6374 는 argus-onboard 가 bind 한다
FOLLOW = (
    "argus_follow_tracker",
    "argus_follow_controller",
    "argus_preview_udp_relay",
    "argus_target_diagnostic",
)


def build(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--build", action="store_true", help="Build/install native ARM runtime")
    mode.add_argument(
        "--host-check", action="store_true", help="Build portable components; no installation"
    )
    parser.add_argument("--build-dir", type=Path)
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args(argv)
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    if args.build and platform.machine() not in ("aarch64", "arm64"):
        parser.error("ARM installation requires aarch64; use --host-check")
    tag = hashlib.sha256(str(ROOT).encode()).hexdigest()[:12]
    cache = (args.build_dir or ROOT / "build/cache" / tag).resolve()
    perception, follow = cache / "perception", cache / "follow"
    dds = "-DARGUS_WITH_DDS=" + ("OFF" if os.environ.get("ARGUS_WITH_DDS") == "0" else "ON")

    def run(command):
        print("+ " + " ".join(map(str, command)), flush=True)
        subprocess.run(list(map(str, command)), check=True, cwd=ROOT)

    run(
        [
            "cmake",
            "-S",
            ROOT,
            "-B",
            perception,
            "-DCMAKE_BUILD_TYPE=Release",
            "-DARGUS_BUILD_GCS=OFF",
            "-DBUILD_TESTING=OFF",
            dds,
            "-DARGUS_BUILD_CAMERA=" + ("ON" if args.build else "OFF"),
            "-DTOML_INCLUDE_DIR=" + str(ROOT / "vendor/pi_deps/include"),
            "-DZMQ_INCLUDE_DIR=" + str(ROOT / "vendor/pi_deps/include"),
            "-DMSGPACK_INCLUDE_DIR=" + str(ROOT / "vendor/yaw/msgpack/include"),
        ]
    )
    products = ["argus-onboard", "argus-frame-source"] + (
        ["i3_managed_capture"] if args.build else []
    )
    run(["cmake", "--build", perception, "--target", *products, "-j", args.jobs])
    run(
        [
            "cmake",
            "-S",
            ROOT / "cmake/follow_b",
            "-B",
            follow,
            "-DBUILD_TESTING=OFF",
            "-DCMAKE_BUILD_TYPE=Release",
            dds,
        ]
    )
    run(["cmake", "--build", follow, "--target", *FOLLOW, "-j", args.jobs])
    if args.host_check:
        run(
            [
                "c++",
                "-std=c++17",
                "-D_LINUX",
                "-I" + str(ROOT / "vendor/camera/sdk_2_12_2/Include"),
                "-c",
                ROOT / "src/argus/perception/camera_capture/i3/capture_managed.cpp",
                "-o",
                cache / "camera_compile_only.o",
            ]
        )
        print("Host build passed; ARM camera linking and hardware execution not performed.")
        return
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if (proc / "exe").resolve(strict=True).is_relative_to(ROOT / "bin"):
                raise RuntimeError("Stop the owned runtime before installation: PID " + proc.name)
        except (FileNotFoundError, PermissionError):
            pass
    stage = Path(tempfile.mkdtemp(prefix=".bin-next-", dir=ROOT))
    try:
        for name in products:
            shutil.copy2(perception / "bin" / name, stage / name)
        for name in FOLLOW:
            shutil.copy2(follow / name, stage / name)
        for file in stage.iterdir():
            elf = file.read_bytes()[:20]
            if elf[:4] != b"\x7fELF" or int.from_bytes(elf[18:20], "little") != 183:
                raise RuntimeError("Not an ARM ELF: " + file.name)
            check = subprocess.run(["ldd", str(file)], capture_output=True, text=True, check=True)
            if "not found" in check.stdout:
                raise RuntimeError(check.stdout)
        previous = None
        if (ROOT / "bin").exists():
            previous = ROOT / "build" / ("previous-bin-" + str(time.time_ns()))
            previous.parent.mkdir(exist_ok=True)
            (ROOT / "bin").rename(previous)
        try:
            stage.rename(ROOT / "bin")
        except BaseException:
            if previous is not None:
                previous.rename(ROOT / "bin")
            raise
    finally:
        if stage.exists():
            shutil.rmtree(stage)
    print(
        f"Installed {len(products) + len(FOLLOW)} ARM executables in bin/. "
        "No process, device or generation state started."
    )


if __name__ == "__main__":
    build()
