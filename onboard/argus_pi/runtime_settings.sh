#!/usr/bin/env bash
# Read one deployment contract. No devices or processes are started here.
argus_settings_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
argus_settings_exports=$(python3 -B "$argus_settings_root/deployment_config.py" --shell --materialize) || return 1
eval "$argus_settings_exports"
unset argus_settings_exports
