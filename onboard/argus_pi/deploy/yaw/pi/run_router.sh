#!/usr/bin/env bash
# Compatibility path for existing remote launchers.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
exec bash "$root/run_router.sh" "$@"
