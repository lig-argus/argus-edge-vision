"""Stable wire topics. Schema versions live inside each message."""

OBSERVATIONS = "perception/observations"
TRACKS = "tracking/tracks"
SYSTEM_STATUS = "system/status"
DIAGNOSTIC_EVENTS = "diagnostics/events"
COMMAND_REQUEST = "command/request"
COMMAND_RESULT = "command/result"
OVERLAY_JPEG = "vision/overlay/jpeg"
NPU_INPUT_JPEG = "vision/npu_input/jpeg"

# Compatibility aliases for the first prototype CLI.
DETECTIONS_V1 = OBSERVATIONS
HEALTH_V1 = DIAGNOSTIC_EVENTS
PREVIEW_JPEG_V1 = OVERLAY_JPEG
