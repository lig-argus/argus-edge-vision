#!/usr/bin/env bash
set -Eeuo pipefail
bundle_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$bundle_root/apps/real_fc"
mkdir -p "$bundle_root/run"
exec 8>"$bundle_root/run/main-gcs-update.lock"
flock -s -n 8 || { echo 'ARGUS update is in progress.' >&2; exit 2; }
export ARGUS_COMMAND_OWNER_FILE="$bundle_root/run/fc-setpoint.owner"
source "$bundle_root/pi_runtime.sh"
require_pi_python "$THESSEN_FC_PYTHON"

# Passive evidence recorder; failure cannot prevent the original FC/perception startup.
if [[ "${1:-}" != "--help" && "${1:-}" != "-h" && "${1:-}" != "--check-config" ]]; then
    bash "$bundle_root/run_flight_recorder.sh" --ensure >/dev/null 2>>"$bundle_root/run/flight-recorder-start.log" || true
    # 2026-11-12 arm 동안 개발 도구(VS Code 서버·rsync·컴파일러) 일시정지 감시기. 실패해도 FC 기동은 계속.
    bash "$bundle_root/run_flight_quiet.sh" >/dev/null 2>>"$bundle_root/run/flight-quiet-start.log" || true
    # 2026-11-16 FC 자세 도착 간격(fc/attitude 구독)·1초 CPU 사용률 수동 기록기. 실패해도 FC 기동은 계속.
    bash "$bundle_root/run_timing_monitor.sh" >/dev/null 2>>"$bundle_root/run/timing-monitor-start.log" || true
fi

# 2026-11-16 FC setpoint 이벤트 송신 설정 (없으면 기존 50ms 재송신).
if [[ -f "$bundle_root/config/gateway/setpoint-timing.env" ]]; then
    set -a
    source "$bundle_root/config/gateway/setpoint-timing.env"
    set +a
fi

# 2026-11-18 탐색 방식 (sector 왕복 / turn 기존 회전). 파일이 없으면 기존 회전.
if [[ -f "$bundle_root/config/gateway/scan.env" ]]; then
    set -a
    source "$bundle_root/config/gateway/scan.env"
    set +a
fi

# 2026-11-06 사용자 확정: FC 연결부를 C++ argus_fc_gateway로 전환 (Python 원본 실행은 run_pi_fc.sh.bak-python-20261106).
exec "$bundle_root/bin/argus_fc_gateway" --router --rc-reclaim --main-gcs "$@"
