from __future__ import annotations

import argparse

import cv2
import numpy as np

from ..transport.MessageBus import DEFAULT_SUBSCRIBE_ENDPOINT, Subscriber
from ..transport.Topics import PREVIEW_JPEG_V1


def main() -> None:
    parser = argparse.ArgumentParser(description="Display ARGUS JPEG preview messages")
    parser.add_argument("--endpoint", default=DEFAULT_SUBSCRIBE_ENDPOINT)
    args = parser.parse_args()
    with Subscriber([PREVIEW_JPEG_V1], args.endpoint, high_water_mark=1) as subscriber:
        while True:
            _, _header, payload = subscriber.receive()
            if payload is None:
                continue
            frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            cv2.imshow("ARGUS vision", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

