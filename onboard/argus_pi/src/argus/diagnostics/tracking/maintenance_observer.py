# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Explicit observation preparation and read-only B target status for maintenance GUI."""

from pathlib import Path
import importlib.util
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "apps/real_fc"))
from tools.real_fc.runtime_settings import settings
from tools.real_fc.follow_proxy import forward_vmy, validate_observation_status
from tools.real_fc.follow_service import prepare_vmy


def emit(**value):
    print(
        json.dumps(
            dict(schema="argus.maintenance.observation.v1", **value),
            ensure_ascii=False,
            allow_nan=False,
        ),
        flush=True,
    )


def main():
    uid = settings()["device"]["expected_fc_uid"]
    try:
        spec = importlib.util.spec_from_file_location(
            "argus_process_admin", ROOT / "src/argus/diagnostics/tracking/maintenance_processes.py"
        )
        admin = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(admin)
        existing = admin.inventory()
        controller_running = any(p["name"] == "argus_follow_controller" for p in existing)
        control_present = any(
            p["name"]
            in (
                "onboard(메인 관제 연결)",
                "IR FC 연결부",
                "RGB/argus_follow_controller",
                "RGB FC 연결부",
            )
            for p in existing
        )
        if controller_running:
            # Existing follow is never restarted, stopped, or reconfigured for observation.
            validate_observation_status(
                forward_vmy(dict(action="status", expected_uid=uid), uid, timeout_ms=350), uid
            )
            emit(ready=True, reason="기존 대상 관리자 상태만 읽습니다", flight_commands=False)
        else:
            if any(p["name"] in ("RGB/argus_follow_controller", "RGB FC 연결부") for p in existing):
                raise ValueError("RGB FC 운용 중에는 IR 프로세스를 새로 시작하지 않습니다")
            if control_present and not admin.confirmed_ground(settings()):
                raise ValueError("기존 FC 제어 경로의 지상·무장 해제 확인 필요")
            result = prepare_vmy(uid)
            emit(
                ready=True,
                reason=result.get("reason", "영상·대상 관리 준비 완료"),
                preview_ready=result.get("preview_ready", False),
                flight_commands=False,
            )
    except Exception as exc:
        emit(ready=False, reason="영상·대상 관리 준비 실패: " + str(exc), flight_commands=False)
        return
    while True:
        try:
            state = validate_observation_status(
                forward_vmy(dict(action="status", expected_uid=uid), uid, timeout_ms=350), uid
            )
            candidates = state.get("candidates")
            if not isinstance(candidates, list):
                raise ValueError("candidates missing")
            emit(
                ready=True,
                candidates=candidates,
                tracking_only=state.get("tracking_only") is True,
                reason=state.get("reason", ""),
                flight_commands=False,
            )
        except BrokenPipeError:
            return
        except Exception as exc:
            try:
                emit(
                    ready=False,
                    candidates=[],
                    reason="대상 상태 확인 실패: " + str(exc),
                    flight_commands=False,
                )
            except BrokenPipeError:
                return
        time.sleep(0.3)


if __name__ == "__main__":
    main()
