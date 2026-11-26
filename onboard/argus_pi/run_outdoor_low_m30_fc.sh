#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ARGUS_FOLLOW_ENVIRONMENT=outdoor
export ARGUS_FOLLOW_CONTROLLER=predictive_low_m30
exec bash "$root/run_pi_fc.sh" "$@"
