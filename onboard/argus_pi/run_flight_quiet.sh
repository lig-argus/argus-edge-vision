#!/usr/bin/env bash
# 2026-11-12 arm 동안 Pi의 개발 도구(VS Code 서버·rsync·컴파일러)를 일시정지하는 감시기를 하나 띄운다.
# 이미 떠 있으면 그대로 둔다. FC에는 아무것도 보내지 않고 게이트웨이 상태(armed)만 읽는다.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$root/pi_runtime.sh"
export PYTHONPATH="$root/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$THESSEN_FC_PYTHON" -B -m argus.runtime.flight_quiet --root "$root" --ensure
