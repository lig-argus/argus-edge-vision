#!/usr/bin/env bash
set -Eeuo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/runtime_common.sh"
make_runtime_args
"$CPP_BIN" "${RUNTIME_ARGS[@]}" --check-config
"$PYTHON_BIN" -m argus_workers.tools.HardwareCheck --profile "${ARGUS_PROFILE:-REAL_IR}" \
  --profiles "$PROJECT_DIR/config/profiles.toml" --frame-source "${ARGUS_FRAME_SOURCE:-auto}" \
  --camera "${1:-${ARGUS_CAMERA:-/dev/video0}}" --hef "${2:-$HEF_PATH}" --labels "${3:-$LABELS_PATH}" \
  --ir-capture "${ARGUS_IR_CAPTURE:-/home/user/thessen_raw14_viewer/build/i3_raw14_capture}"
