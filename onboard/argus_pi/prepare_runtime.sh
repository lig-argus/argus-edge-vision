#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:---check}" in
 --check) shift || true; exec python3 -B "$root/check_target.py" "$@" ;;
 --build|--host-check) exec bash "$root/build_runtime.sh" "$@" ;;
 *) echo 'Usage: prepare_runtime.sh --check | --build | --host-check'; exit 2 ;;
esac
