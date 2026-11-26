#!/usr/bin/env bash
# Observer configuration only; never changes camera, tracking or control settings.
argus_flight_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ARGUS_FLIGHT_RECORD=0
if [[ -f "$argus_flight_root/config/flight_recording.json" ]]; then
    argus_record_enabled=$(python3 -B -c 'import json,sys;print(1 if json.load(open(sys.argv[1])).get("enabled") is True else 0)' "$argus_flight_root/config/flight_recording.json") || argus_record_enabled=0
    export ARGUS_FLIGHT_RECORD="$argus_record_enabled"
fi
mkdir -p "$argus_flight_root/run/flight-recorder"
export ARGUS_FLIGHT_TAP_ENDPOINT="ipc://$argus_flight_root/run/flight-recorder/pixels.sock"
export ARGUS_FLIGHT_CMC_LOG="$argus_flight_root/run/flight-recorder/cmc-stream.jsonl"
unset argus_flight_root argus_record_enabled
