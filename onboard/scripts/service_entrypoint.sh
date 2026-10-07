#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/runtime_common.sh"
make_runtime_args
case "${1:-run}" in
  check) "$CPP_BIN" "${RUNTIME_ARGS[@]}" --check-config ;;
  run) exec "$CPP_BIN" "${RUNTIME_ARGS[@]}" ;;
  *) echo 'service_entrypoint.sh run|check' >&2; exit 2 ;;
esac
