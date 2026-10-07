#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$PROJECT_DIR/.venv-rpi/bin/python" ]]; then
  python3 -m venv --system-site-packages "$PROJECT_DIR/.venv-rpi"
fi
"$PROJECT_DIR/.venv-rpi/bin/python" -m pip install --no-deps --no-build-isolation -e "$PROJECT_DIR"
bash "$PROJECT_DIR/scripts/build.sh"
echo 'Existing HailoRT and SDK retained. See docs/CPP_MIGRATION.md for dependencies.'
