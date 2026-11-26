#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$root/runtime_settings.sh"
if [[ "${1:-}" == "--help" ]]; then
    cat <<'USAGE'
Usage: bash run_follow_b_tracker.sh [tracker options]
Environment:
  ARGUS_FOLLOW_B_BUILD_DIR         Default: <repository>/bin
  ARGUS_FOLLOW_B_CAMERA_PROFILE    Default: <repository>/config/device/camera.json
  ARGUS_FOLLOW_B_FC_STATE_ENDPOINT Default: tcp://127.0.0.1:6370
  ARGUS_OBSERVATIONS_ENDPOINT     Default: tcp://127.0.0.1:6374
Publishes tracking/tracks and tracking/status (team ZeroMQ + protobuf) on endpoints.tracks
(config/runtime.json, default tcp://127.0.0.1:6375) unless --output overrides it.
Default --geometry auto: FC-ground when valid, static-camera tracking-only otherwise.
  --geometry camera              Never opens an FC telemetry socket.
  --geometry fc-ground           Retain strict FC-dependent tracking.
Camera-only tracks cannot start vehicle following; no fabricated FC pose/NED output.
USAGE
    exit 0
fi
build_dir="${ARGUS_FOLLOW_B_BUILD_DIR:-$root/bin}"
binary="$build_dir/argus_follow_tracker"
if [[ ! -x "$binary" ]]; then
    printf 'B tracker executable not found: %s\nRun bash "%s/build_runtime.sh" --build first. No legacy fallback.\n' "$binary" "$root" >&2
    exit 1
fi
uid="$ARGUS_EXPECTED_FC_UID"
if [[ ! "$uid" =~ ^[0-9a-fA-F]{36}$ ]]; then
    printf 'Invalid expected FC UID in yaw-only.config; refusing B tracker startup.\n' >&2
    exit 1
fi

# Passive evidence recorder; failure cannot prevent the original FC/perception startup.
if [[ "${1:-}" != "--help" && "${1:-}" != "-h" && "${1:-}" != "--check-config" ]]; then
    bash "$root/run_flight_recorder.sh" --ensure >/dev/null 2>>"$root/run/flight-recorder-start.log" || true
fi
source "$root/flight_record_environment.sh"
# 탐지 결과 수신 방식: zmq(기본, 2026-11-16 DDS 미도입) | dds (dds는 argus-onboard의 ARGUS_DDS_PUBLISH=1 필요)
export ARGUS_OBSERVATION_TRANSPORT="${ARGUS_OBSERVATION_TRANSPORT:-zmq}"
export ARGUS_DDS_PROFILE_FILE="${ARGUS_DDS_PROFILE_FILE:-$root/config/dds/argus_dds_profiles.xml}"
# 2026-11-12 임시 카메라 자세 보정 스위치 (camera_pitch_correction.sh on|off|status).
# 파일이 없거나 꺼져 있으면 기존 동작(배율 1.0, 시각 0ms).
correction_file="$root/config/tracking/attitude-correction.env"
if [[ -f "$correction_file" ]]; then
    # shellcheck disable=SC1090
    source "$correction_file"
fi
export ARGUS_CAMERA_PITCH_SCALE="${ARGUS_CAMERA_PITCH_SCALE:-1.0}"
export ARGUS_CAMERA_ATTITUDE_OFFSET_MS="${ARGUS_CAMERA_ATTITUDE_OFFSET_MS:-0}"
# 2026-11-16 FC 자세 외삽: 노출 뒤 FC 자세가 아직 없으면 기다리지 않고 직전 자세+각속도로 외삽.
# 파일이 없거나 0이면 꺼짐(기존 대기). 켜기: ARGUS_ATTITUDE_EXTRAPOLATION_MS=25 (0..33)
extrapolation_file="$root/config/tracking/attitude-extrapolation.env"
if [[ -f "$extrapolation_file" ]]; then
    # shellcheck disable=SC1090
    source "$extrapolation_file"
fi
export ARGUS_ATTITUDE_EXTRAPOLATION_MS="${ARGUS_ATTITUDE_EXTRAPOLATION_MS:-0}"
export ARGUS_ATTITUDE_EXTRAPOLATION_MAX_RATE_DPS="${ARGUS_ATTITUDE_EXTRAPOLATION_MAX_RATE_DPS:-40}"

exec "$binary" \
    --calibration "$ARGUS_CAMERA_PROFILE" \
    --measurement "$ARGUS_MEASUREMENT_PROFILE" \
    --ucmc-profile "$ARGUS_UCMC_PROFILE" \
    --camera-only-calibration "${ARGUS_CAMERA_ONLY_CALIBRATION:-$root/config/ucmc-camera-estimate.txt}" \
    --expected-uid "$uid" \
    --fc-state-endpoint "${ARGUS_FOLLOW_B_FC_STATE_ENDPOINT:-tcp://127.0.0.1:6370}" \
    --receipt-retry-ms "${ARGUS_V21_RECEIPT_RETRY_MS:-0}" \
    --output "$ARGUS_TRACKS_ENDPOINT" \
    --input "$ARGUS_OBSERVATIONS_ENDPOINT" "$@"
