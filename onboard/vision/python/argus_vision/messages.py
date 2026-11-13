# 탐지 값 타입을 protobuf 메시지로 바꾸는 함수.
# 정의: onboard/libs/protocol/proto/argus_msg/ (process_interfaces.md 3장.
# 토픽마다 .proto 하나, 함께 쓰는 탐지 결과는 detection.proto).

from collections.abc import Sequence

from argus_msg import detection_pb2

from argus_vision.types import Detection


def detection_messages(
    detections: Sequence[Detection],
) -> list[detection_pb2.Detection]:
    """탐지 결과를 메시지로 바꾼다. 좌표는 정수 픽셀.

    Args:
        detections: 원본 영상 픽셀 좌표의 탐지 결과.

    Returns:
        같은 순서의 `argus_msg.Detection` 목록.
    """
    return [
        detection_pb2.Detection(
            class_id=d.classification.class_id,
            label=d.classification.label,
            confidence=d.classification.confidence,
            x1=int(d.box.x1),
            y1=int(d.box.y1),
            x2=int(d.box.x2),
            y2=int(d.box.y2),
        )
        for d in detections
    ]
