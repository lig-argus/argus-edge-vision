#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ARGUS_FOLLOW_ENVIRONMENT=outdoor
export ARGUS_FOLLOW_CONTROLLER=predictive_low
exec bash "$root/run_follow_b_controller.sh" "$@"
