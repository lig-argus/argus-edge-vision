# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Offline effective-profile verification; no sockets, SDK or flight commands."""

from pathlib import Path
import importlib.util
import json
import subprocess
import sys

root = Path(sys.argv[1]).resolve()
checker = Path(sys.argv[2]).resolve()
spec = importlib.util.spec_from_file_location(
    "argus_effective_profiles", root / "deployment_config.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
profiles = []
# 2026-11-18: 예전 'pid' 항목은 더 이상 유효하지 않다. 실외 4 m는 predictive뿐이라
# (low/m30 계열은 3 m) 실외 predictive의 이동/회전 두 프로파일을 검사한다.
# 검사기는 첫 번째 = 실내, 나머지 = 실외 4 m를 요구한다.
for environment, controller, mode in [
    ("indoor", "predictive", "moving"),
    ("outdoor", "predictive", "moving"),
    ("outdoor", "predictive", "yaw"),
]:
    data = module.materialize(
        module.load(
            root, {"ARGUS_FOLLOW_ENVIRONMENT": environment, "ARGUS_FOLLOW_CONTROLLER": controller}
        )
    )
    profiles.append(data["paths"][mode])
result = subprocess.run([str(checker), *profiles], check=False)
# 2026-11-18 실내 데모 모드: 이동/회전 칸이 같은 demo-yaw.config, 실내 측정/카메라, 회전 전용 프로파일.
demo_errors = []
demo = module.load(root, {"ARGUS_FOLLOW_ENVIRONMENT": "indoor", "ARGUS_FOLLOW_CONTROLLER": "demo"})
expected = (root / "config/follow/demo-yaw.config").resolve()
if Path(demo["paths"]["moving"]) != expected or Path(demo["paths"]["yaw"]) != expected:
    demo_errors.append("demo moving/yaw path")
if Path(demo["paths"]["measurement"]) != (root / "config/tracking/measurement.json").resolve():
    demo_errors.append("demo measurement path")
if Path(demo["paths"]["camera"]) != (root / "config/device/camera.json").resolve():
    demo_errors.append("demo camera path")
demo = module.materialize(demo)
for mode in ("moving", "yaw"):
    values = dict(
        l.split("=", 1)
        for l in Path(demo["paths"][mode]).read_text(encoding="utf8").splitlines()
        if "=" in l and not l.lstrip().startswith("#")
    )
    if values.get("yaw_only") != "true" or values.get("follow_controller_profile") != "demo":
        demo_errors.append("demo " + mode + " profile is not yaw_only/demo")
for env in [
    {"ARGUS_FOLLOW_ENVIRONMENT": "outdoor", "ARGUS_FOLLOW_CONTROLLER": "demo"},
    {"ARGUS_FOLLOW_ENVIRONMENT": "indoor", "ARGUS_FOLLOW_CONTROLLER": "predictive_low"},
]:
    try:
        module.load(root, env)
        demo_errors.append(
            "accepted " + env["ARGUS_FOLLOW_ENVIRONMENT"] + "/" + env["ARGUS_FOLLOW_CONTROLLER"]
        )
    except ValueError:
        pass
for error in demo_errors:
    print("demo check failed:", error, file=sys.stderr)
code = result.returncode or (5 if demo_errors else 0)
if len(sys.argv) > 3:
    Path(sys.argv[3]).write_text(
        json.dumps(
            {
                "profiles": profiles,
                "demo_profile": demo["paths"]["moving"],
                "checker_exit": result.returncode,
                "demo_errors": demo_errors,
                "exit": code,
                "device_or_socket_opened": False,
            },
            indent=2,
        ),
        encoding="utf8",
    )
raise SystemExit(code)
