#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ARGUS_FOLLOW_B_CAMERA_PROFILE="$root/config/device/camera-rgb.json"
exec bash "$root/run_follow_b_controller.sh" "$@"
