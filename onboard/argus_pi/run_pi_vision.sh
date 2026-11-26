#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$root/runtime_settings.sh"
# An explicit live viewer connection prepares the Pi producer; virtual tests do not.
if [[ "${1:-}" == --ensure-producer ]]; then
    shift
    source "$root/pi_runtime.sh"
    require_pi_python "$THESSEN_PERCEPTION_PYTHON"
    "$THESSEN_PERCEPTION_PYTHON" -B \
        "$root/src/argus/runtime/streaming/preview_producer_supervisor.py" --root "$root"
fi
exec "$ARGUS_BIN_DIR/argus_preview_udp_relay" \
    --endpoint "$ARGUS_VISION_ENDPOINT" "$@"
