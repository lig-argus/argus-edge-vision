# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Staged readiness checks; never open a device or initialize identity state."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess

from deployment_config import ROOT, load
from setup_environment import probe, resolve_python

BINARIES = (
    "argus-onboard",
    "argus-frame-source",
    "i3_managed_capture",
    "argus_follow_tracker",
    "argus_follow_controller",
    "argus_preview_udp_relay",
)


def identity(path):
    path = Path(path)
    if not path.is_absolute() or path.is_symlink():
        raise ValueError("Identity path must be absolute and must not be a symlink")
    fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    try:
        data = os.read(fd, 129)
    finally:
        os.close(fd)
    match = re.fullmatch(rb"ARGUS_FLOAT_IDS_V1\n([0-9]+)\n", data)
    if not match or len(data) >= 128 or not 0 <= int(match[1]) < 16777215:
        raise ValueError("Generation missing, corrupt or exhausted; explicit recovery required")
    return "valid; unchanged"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stage", choices=["all", "config", "dependencies", "binaries", "identity"], default="all"
    )
    parser.add_argument(
        "--host-check",
        action="store_true",
        help="Portable-only checks; cannot claim ARM/hardware readiness",
    )
    parser.add_argument("--binary-dir", type=Path, default=ROOT / "bin")
    parser.add_argument("--perception-dir", type=Path)
    parser.add_argument(
        "--identity-file",
        type=Path,
        default=Path(
            os.environ.get(
                "ARGUS_TRACKING_ID_STATE",
                str(Path.home() / ".local/state/argus/tracking-ids/generation"),
            )
        ),
    )
    args = parser.parse_args(argv)
    rows = []

    def check(group, name, operation):
        try:
            value = operation()
            rows.append({"stage": group, "name": name, "ok": True, "detail": value})
        except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
            rows.append({"stage": group, "name": name, "ok": False, "detail": str(exc)})

    if args.stage in ("all", "config"):
        check("config", "deployment", lambda: load())

        def model_check():
            values = {}
            for line in (ROOT / "config/runtime.env").read_text(encoding="utf-8").splitlines():
                if "=" in line and not line.lstrip().startswith("#"):
                    key, value = line.split("=", 1)
                    values[key] = shlex.split(value)[0]
            manifest = json.loads((ROOT / "assets/model.json").read_text())
            model = (ROOT / values["ARGUS_HEF"]).resolve()
            if model != (ROOT / manifest["path"]).resolve():
                raise ValueError("Model changed; update and validate assets/model.json explicitly")
            if hashlib.sha256(model.read_bytes()).hexdigest() != manifest["sha256"]:
                raise ValueError("Model SHA256 mismatch")
            labels = (ROOT / values["ARGUS_LABELS"]).read_text().splitlines()
            if labels != manifest["classes"]:
                raise ValueError("Model/label class mismatch")
            return {"model": str(model), "sha256": manifest["sha256"], "classes": labels}

        check("config", "model-label-integrity", model_check)
    if args.stage in ("all", "dependencies"):
        for role, name, variable in [
            ("fc", ".venv-fc", "THESSEN_FC_PYTHON"),
            ("vision", ".venv-rpi", "THESSEN_PERCEPTION_PYTHON"),
        ]:

            def environment(role=role, name=name, variable=variable):
                result = probe(resolve_python(role), role)
                if not result["ok"]:
                    raise ValueError(result["detail"])
                return result

            check("dependencies", role + "-python", environment)
        for tool in ["cmake", "c++", "mavlink-routerd", "flock", "fuser"]:

            def executable(tool=tool):
                value = shutil.which(tool)
                if value is None:
                    raise ValueError("Missing executable: " + tool)
                return value

            check("dependencies", tool, executable)
    if args.stage in ("all", "binaries"):
        for name in BINARIES:
            if args.host_check and name == "i3_managed_capture":
                rows.append(
                    {
                        "stage": "binaries",
                        "name": name,
                        "ok": None,
                        "detail": "ARM SDK link not tested on host",
                    }
                )
                continue

            def binary(name=name):
                folder = (
                    args.perception_dir
                    if args.perception_dir and name in ("argus-onboard", "i3_managed_capture")
                    else args.binary_dir
                )
                path = folder / name
                with path.open("rb") as stream:
                    elf = stream.read(20)
                if len(elf) < 20 or elf[:4] != b"\x7fELF":
                    raise ValueError("Not an ELF executable: " + str(path))
                machine = int.from_bytes(elf[18:20], "little")
                expected = 62 if args.host_check and platform.machine() == "x86_64" else 183
                if machine != expected:
                    raise ValueError("Wrong binary architecture")
                result = subprocess.run(
                    ["ldd", str(path)], capture_output=True, text=True, check=True, timeout=20
                )
                if "not found" in result.stdout:
                    raise ValueError(result.stdout)
                return {"path": str(path), "elf_machine": machine, "libraries_resolved": True}

            check("binaries", name, binary)
    if args.stage in ("all", "identity"):
        check("identity", "persistent-generation", lambda: identity(args.identity_file))
    passed = all(row["ok"] is not False for row in rows)
    report = {
        "checks_passed": passed,
        "scope": args.stage,
        "host_check": args.host_check,
        "deployment_ready": passed
        and args.stage == "all"
        and not args.host_check
        and platform.machine() == "aarch64",
        "hardware_tested": False,
        "flight_tested": False,
        "generation_modified": False,
        "checks": rows,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
