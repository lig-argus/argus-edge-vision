#!/usr/bin/env bash
# Compatibility name; the build is no longer host-specific.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$root/build_runtime.sh" --build "$@"
