# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Explicit offline environment setup. No OS changes, devices, FC commands or ID initialization."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import venv

ROOT = Path(__file__).resolve().parent
MODULES = {
    "fc": ["pymavlink", "serial", "zmq", "msgpack"],
    "vision": ["numpy", "cv2", "zmq", "msgpack", "netifaces", "hailo_platform"],
}
ENV_NAMES = {"fc": ".venv-fc", "vision": ".venv-rpi"}


def resolve_python(role, root=ROOT):
    variable = "THESSEN_FC_PYTHON" if role == "fc" else "THESSEN_PERCEPTION_PYTHON"
    command = 'source "$1/pi_runtime.sh"; printf "%s" "${' + variable + '}"'
    result = subprocess.run(
        ["bash", "-c", command, "environment", str(root)],
        capture_output=True,
        text=True,
        check=True,
    )
    if not result.stdout:
        raise ValueError("Python path is empty: " + role)
    return Path(result.stdout)


def probe(python, role):
    code = (
        "import importlib,importlib.metadata as m,json,sys; names="
        + repr(MODULES[role])
        + "; mods=[importlib.import_module(n) for n in names]; "
        'print(json.dumps({"python":sys.version,"modules":{n:v.__file__ for n,v in zip(names,mods)}}))'
    )
    try:
        result = subprocess.run(
            [str(python), "-B", "-c", code], capture_output=True, text=True, timeout=30
        )
        return {
            "ok": result.returncode == 0,
            "python": str(python),
            "detail": result.stdout.strip() if result.returncode == 0 else result.stderr.strip(),
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"ok": False, "python": str(python), "detail": str(exc)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="Read-only import checks (default)")
    mode.add_argument(
        "--create", action="store_true", help="Create new internal venv; never overwrite one"
    )
    parser.add_argument("--role", choices=["fc", "vision", "all"], default="all")
    parser.add_argument(
        "--wheelhouse", type=Path, help="Pre-acquired CPython3.12/aarch64 wheels; offline only"
    )
    args = parser.parse_args(argv)
    roles = list(MODULES) if args.role == "all" else [args.role]
    if args.create:
        if platform.machine() != "aarch64" or sys.version_info[:2] != (3, 12):
            parser.error(
                "Target setup requires aarch64/Python3.12; existing host checks remain available"
            )
        if args.wheelhouse is None or not args.wheelhouse.is_dir():
            parser.error("--create requires an existing offline --wheelhouse")
        if "vision" in roles:
            manifest = json.loads((ROOT / "requirements/platform.json").read_text())[
                "hailort_wheel"
            ]
            wheel = args.wheelhouse / manifest["filename"]
            if (
                not wheel.is_file()
                or hashlib.sha256(wheel.read_bytes()).hexdigest() != manifest["sha256"]
            ):
                parser.error("Required validated HailoRT wheel missing or hash mismatch")
        for role in roles:
            target = ROOT / ENV_NAMES[role]
            if target.exists() or target.is_symlink():
                parser.error("Existing environment is preserved; use --check: " + str(target))
        for role in roles:
            target = ROOT / ENV_NAMES[role]
            try:
                venv.EnvBuilder(system_site_packages=(role == "vision"), with_pip=True).create(
                    target
                )
                subprocess.run(
                    [
                        str(target / "bin/python"),
                        "-m",
                        "pip",
                        "install",
                        "--no-index",
                        "--find-links",
                        str(args.wheelhouse.resolve()),
                        "--no-deps",
                        "-r",
                        str(ROOT / "requirements" / (role + ".lock")),
                    ],
                    check=True,
                )
                site = target / "lib/python3.12/site-packages"
                (site / "argus_runtime.pth").write_text(
                    str(ROOT / "src") + "\n" + str(ROOT / "apps/real_fc") + "\n"
                )
                result = probe(target / "bin/python", role)
                if not result["ok"]:
                    raise RuntimeError(result["detail"])
            except BaseException:
                if target.exists():
                    target.rename(ROOT / (ENV_NAMES[role] + ".failed-" + str(time.time_ns())))
                raise
    report = {}
    for role in roles:
        try:
            report[role] = probe(resolve_python(role), role)
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            report[role] = {"ok": False, "detail": str(exc)}
    print(
        json.dumps(
            {"roles": report, "hardware_opened": False, "state_initialized": False}, indent=2
        )
    )
    return 0 if all(item["ok"] for item in report.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
