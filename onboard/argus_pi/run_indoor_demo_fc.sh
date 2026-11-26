#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# 2026-11-18 실내 데모 모드: 회전 추종만, 고도 1.5 m 유지 (1.0~2.0 m).
export ARGUS_FOLLOW_ENVIRONMENT=indoor
# 게이트웨이는 ARGUS_FOLLOW_CONTROLLER를 실외 제어기 기본값으로 읽으므로 실내 값은 따로 준다.
export ARGUS_FOLLOW_INDOOR_CONTROLLER=demo
exec bash "$root/run_pi_fc.sh" "$@"
