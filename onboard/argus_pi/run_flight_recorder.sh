#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$root/pi_runtime.sh"
export PYTHONPATH="$root/src${PYTHONPATH:+:$PYTHONPATH}"
if [[ "${1:-}" == "--check" ]]; then
    exec "$THESSEN_FC_PYTHON" -B -m argus.diagnostics.flight_record.daemon --root "$root" --check
fi
exec "$THESSEN_FC_PYTHON" -B -m argus.diagnostics.flight_record.manager --root "$root" --ensure
