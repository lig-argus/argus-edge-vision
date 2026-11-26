#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$root/runtime_settings.sh"
if [[ "${1:-}" == "--help" ]]; then
    cat <<'USAGE'
Usage: bash run_follow_b_controller.sh [controller options]
  --check-config                  Validate profiles without opening FC or ZeroMQ connections.
  --mode yaw|moving               Initial guidance mode; default yaw.
  --legacy-mode-rpc               Explicit old RPC mode; default uses PR26 mode events.
  --scan-rate DEG_S               Process default scan rate; default 10.
  --scan-timeout-ms MS            Process default scan timeout; default 60000.
Environment:
  ARGUS_FOLLOW_B_BUILD_DIR         Default: <repository>/bin
  ARGUS_FOLLOW_B_CAMERA_PROFILE    Default: <repository>/config/device/camera.json
  ARGUS_FOLLOW_B_FC_STATE_ENDPOINT Default: tcp://127.0.0.1:6370
  ARGUS_FOLLOW_B_ONBOARD_RPC_ENDPOINT Default: tcp://127.0.0.1:6371
  ARGUS_FOLLOW_B_CONTROL_ENDPOINT  Default: tcp://127.0.0.1:6372
  ARGUS_FOLLOW_B_OPERATOR_ENDPOINT Default: tcp://127.0.0.1:6373
Requires the separate onboard FC owner. Never starts a router, FC gateway, or legacy controller.
USAGE
    exit 0
fi
build_dir="${ARGUS_FOLLOW_B_BUILD_DIR:-$root/bin}"
binary="$build_dir/argus_follow_controller"
if [[ ! -x "$binary" ]]; then
    printf 'B controller executable not found: %s\nRun bash "%s/build_runtime.sh" --build first. No legacy fallback.\n' "$binary" "$root" >&2
    exit 1
fi
# 2026-11-09: 재시작하면 control.csv를 새로 쓰므로 직전 기록을 먼저 보관한다 (최근 30개 유지).
backup_control_csv() {
    local csv="" profile
    for profile in "${ARGUS_YAW_PROFILE:-}" "${ARGUS_MOVING_PROFILE:-}"; do
        if [[ -z "$csv" && -f "$profile" ]]; then
            csv="$(sed -n "s/^diagnostics_csv=//p" "$profile" | tail -n 1)"
        fi
    done
    [[ -n "$csv" ]] || return 0
    [[ "$csv" == /* ]] || csv="$PWD/$csv"
    local archive="$root/run/follow_b/control-archive"
    local target
    target="$archive/$(date +%Y%m%d-%H%M%S)-$$"
    local file copied=0
    for file in "$csv" "$csv.target.csv"; do
        if [[ -s "$file" ]]; then
            mkdir -p "$target" && cp -p "$file" "$target/" && copied=1
        fi
    done
    if [[ "$copied" == 1 ]]; then
        echo "control.csv 보관: $target" >&2
        ls -1dt "$archive"/*/ 2>/dev/null | tail -n +31 | xargs -r rm -rf --
    fi
}
backup_control_csv || echo "control.csv 보관 실패 (기동은 계속)" >&2
# 2026-11-16 제어 이벤트 구동: 새 추적 결과가 오면 바로 제어 tick (최소 20ms, 없으면 기존 주기).
# 파일이 없거나 0이면 꺼짐(기존 고정 주기). 켜기: ARGUS_CONTROL_EVENT_DRIVEN=1, 바꾼 뒤 제어기 재시작.
timing_file="$root/config/follow/control-timing.env"
if [[ -f "$timing_file" ]]; then
    # shellcheck disable=SC1090
    source "$timing_file"
fi
export ARGUS_CONTROL_EVENT_DRIVEN="${ARGUS_CONTROL_EVENT_DRIVEN:-0}"
exec "$binary" \
    --moving-profile "$ARGUS_MOVING_PROFILE" \
    --yaw-profile "$ARGUS_YAW_PROFILE" \
    --camera-profile "$ARGUS_CAMERA_PROFILE" \
    --fc-state-endpoint "${ARGUS_FOLLOW_B_FC_STATE_ENDPOINT:-tcp://127.0.0.1:6370}" \
    --onboard-rpc-endpoint "${ARGUS_FOLLOW_B_ONBOARD_RPC_ENDPOINT:-tcp://127.0.0.1:6371}" \
    --control-pub-endpoint "${ARGUS_FOLLOW_B_CONTROL_ENDPOINT:-tcp://127.0.0.1:6372}" \
    --operator-endpoint "${ARGUS_FOLLOW_B_OPERATOR_ENDPOINT:-tcp://127.0.0.1:6373}" "$@"
