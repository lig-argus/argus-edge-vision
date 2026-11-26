#!/usr/bin/env bash
# Package-only: optionally reuse target-specific Python paths.
pi_env_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$pi_env_root/config/pi_environment.sh" ]]; then source "$pi_env_root/config/pi_environment.sh"; fi
# 기존 Pi 환경을 역할별로 재사용한다. 이 파일은 설치하거나 프로세스를 시작하지 않는다.
THESSEN_PERCEPTION_PYTHON="${THESSEN_PERCEPTION_PYTHON:-$pi_env_root/.venv-rpi/bin/python}"
THESSEN_YAW_PYTHON="${THESSEN_YAW_PYTHON:-$THESSEN_PERCEPTION_PYTHON}"
argus_runtime_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
THESSEN_FC_PYTHON="${THESSEN_FC_PYTHON:-$argus_runtime_root/.venv-fc/bin/python}"
THESSEN_MAVSDK_PREFIX="${THESSEN_MAVSDK_PREFIX:-${MAVSDK_PREFIX:-/usr/local}}"
export THESSEN_PERCEPTION_PYTHON THESSEN_YAW_PYTHON THESSEN_FC_PYTHON THESSEN_MAVSDK_PREFIX

require_pi_python() {
    [[ -x "$1" ]] || { printf '%s\n' "Existing Pi Python not found: $1" >&2; return 1; }
}
