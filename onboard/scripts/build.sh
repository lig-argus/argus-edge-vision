#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$PROJECT_DIR/build" -j"${ARGUS_BUILD_JOBS:-2}"
ctest --test-dir "$PROJECT_DIR/build" --output-on-failure
