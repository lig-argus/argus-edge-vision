#!/usr/bin/env bash
# Shared launcher configuration. C++ CompositionRoot validates profiles.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="$PROJECT_DIR/.venv-rpi/bin/python"
CPP_BIN="$PROJECT_DIR/build/bin/argus-onboard"
if [[ -f "$PROJECT_DIR/config/runtime.env" ]]; then
  set -a
  source "$PROJECT_DIR/config/runtime.env"
  set +a
fi
export PYTHONPATH="$PROJECT_DIR/python${PYTHONPATH:+:$PYTHONPATH}"
resolve_project_path() {
  if [[ "$1" = /* ]]; then printf '%s\n' "$1"; else printf '%s\n' "$PROJECT_DIR/$1"; fi
}
make_runtime_args() {
  : "${ARGUS_HEF:?Set ARGUS_HEF in config/runtime.env}"
  [[ -x "$CPP_BIN" && -x "$PYTHON_BIN" ]] || { echo "Build C++ and install Python workers first." >&2; return 1; }
  HEF_PATH="$(resolve_project_path "$ARGUS_HEF")"
  LABELS_PATH="$(resolve_project_path "${ARGUS_LABELS:-config/labels.txt}")"
  [[ -f "$HEF_PATH" && -f "$LABELS_PATH" ]] || { echo "HEF/labels missing." >&2; return 1; }
  RUNTIME_ARGS=(--profile "${ARGUS_PROFILE:-REAL_IR}" --profiles "$PROJECT_DIR/config/profiles.toml"
    --frame-source "${ARGUS_FRAME_SOURCE:-auto}" --camera "${ARGUS_CAMERA:-/dev/video0}"
    --backend "${ARGUS_BACKEND:-auto}" --python "$PYTHON_BIN" --hef "$HEF_PATH" --labels "$LABELS_PATH"
    --width "${ARGUS_WIDTH:-640}" --height "${ARGUS_HEIGHT:-480}" --fps "${ARGUS_FPS:-30}" --fourcc "${ARGUS_FOURCC:-MJPG}"
    --frame-queue-size "${ARGUS_FRAME_QUEUE_SIZE:-2}" --score-threshold "${ARGUS_SCORE_THRESHOLD:-0.35}"
    --ir-capture "${ARGUS_IR_CAPTURE:-/home/user/thessen_raw14_viewer/build/i3_raw14_capture}"
    --ir-device "${ARGUS_IR_DEVICE:-0}" --ir-timeout "${ARGUS_IR_TIMEOUT:-15}" --ir-preprocess "${ARGUS_IR_PREPROCESS:-minmax}"
    --log-every "${ARGUS_LOG_EVERY:-30}" --preview-max-fps "${ARGUS_PREVIEW_MAX_FPS:-10}"
    --jpeg-quality "${ARGUS_JPEG_QUALITY:-75}" --worker-timeout-ms "${ARGUS_WORKER_TIMEOUT_MS:-10000}"
    --npu-input-max-fps "${ARGUS_NPU_INPUT_MAX_FPS:-15}" --npu-input-jpeg-quality "${ARGUS_NPU_INPUT_JPEG_QUALITY:-80}")
  [[ "${ARGUS_PUBLISH_PREVIEW:-0}" != 1 ]] || RUNTIME_ARGS+=(--publish-preview)
  [[ "${ARGUS_PUBLISH_NPU_INPUT:-0}" != 1 ]] || RUNTIME_ARGS+=(--publish-npu-input)
  [[ -z "${ARGUS_LATENCY_CSV:-}" ]] || RUNTIME_ARGS+=(--latency-csv "$(resolve_project_path "$ARGUS_LATENCY_CSV")")
  return 0
}
