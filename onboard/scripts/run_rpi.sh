#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/runtime_common.sh"
if [[ $# -gt 4 ]]; then echo 'run_rpi.sh [HEF] [CAMERA] [LABELS] [--preview]' >&2; exit 2; fi
ARGUS_HEF="${1:-${ARGUS_HEF:-}}"
ARGUS_CAMERA="${2:-${ARGUS_CAMERA:-/dev/video0}}"
ARGUS_LABELS="${3:-${ARGUS_LABELS:-config/labels.txt}}"
if [[ -n "${4:-}" && "$4" != --preview ]]; then echo 'Fourth argument must be --preview' >&2; exit 2; fi
make_runtime_args
if [[ "${4:-}" == --preview ]]; then RUNTIME_ARGS+=(--publish-preview); fi
BUS_PID=""
VIEWER_PID=""
cleanup() {
  for pid in "$VIEWER_PID" "$BUS_PID"; do
    if [[ -n "$pid" ]]; then kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT INT TERM
"$PYTHON_BIN" -m argus_workers.transport.BusProxy &
BUS_PID=$!
sleep 0.5
kill -0 "$BUS_PID" || { echo 'Bus failed to start; inspect ports 5555/5556.' >&2; exit 1; }
if [[ "${4:-}" == --preview && -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  "$PYTHON_BIN" -m argus_workers.tools.PreviewSubscriber &
  VIEWER_PID=$!
fi
echo 'Starting C++ argus-onboard (Python image workers). Stop: Ctrl+C'
"$CPP_BIN" "${RUNTIME_ARGS[@]}"
