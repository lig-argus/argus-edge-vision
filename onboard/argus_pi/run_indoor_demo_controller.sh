#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# 2026-11-18 실내 데모 모드: 회전 추종만, 고도 1.5 m 유지 (1.0~2.0 m).
export ARGUS_FOLLOW_ENVIRONMENT=indoor
export ARGUS_FOLLOW_CONTROLLER=demo
export ARGUS_FOLLOW_INDOOR_CONTROLLER=demo
exec bash "$root/run_follow_b_controller.sh" "$@"
