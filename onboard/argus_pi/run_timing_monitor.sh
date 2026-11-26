#!/usr/bin/env bash
# 2026-11-16 FC 자세 도착 간격(fc/attitude 6370 구독)과 1초 CPU 사용률을 기록하는 수동 감시기를 하나 띄운다.
# 이미 떠 있으면 그대로 둔다. FC에는 아무것도 보내지 않는다. 결과: run/timing-monitor/<시작시각>/
# 분석: python3 src/argus/diagnostics/timing/timing_report.py run/timing-monitor/<시작시각>
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$root/pi_runtime.sh"
exec "$THESSEN_FC_PYTHON" -B "$root/src/argus/diagnostics/timing/timing_monitor.py" --root "$root" --ensure "$@"
