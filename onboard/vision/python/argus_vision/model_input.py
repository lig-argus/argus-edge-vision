# 흑백 프레임으로 HEF 입력을 만들고, 모델 좌표의 박스를 원본 영상 픽셀로 되돌린다.
# 출처: 실기체 RPi `preprocessor.py` 의 letterbox_bgr_to_rgb, LetterboxTransform 과
#   `prep_fast.py` 의 decode_letterbox_fast, letterbox_gray_to_rgb
#   (같은 결과의 빠른 구현).
# - 1채널 HEF 이고 크기가 같으면(지금 실기체 640×480) 흑백 영상을
#   복사 없이 그대로 넣는다.
# - 크기가 다르면 Hailo 예제와 같은 레터박스(INTER_CUBIC, 회색 114 채움)를 한다.
#   640×480 → 640×640 은 배율이 1 이라 크기 변경 없이 위아래만 채운다.
# - 흑백은 BGR 과 RGB 가 같아서 채널 순서를 바꾸지 않는다.

from dataclasses import dataclass

import cv2
import numpy as np

from argus_vision.arrays import Gray8Image
from argus_vision.types import Gray8Frame, PixelBox

LETTERBOX_PAD_VALUE = 114


@dataclass(frozen=True, slots=True)
class LetterboxTransform:
    """원본 영상과 모델 입력 사이의 배율과 여백.

    Attributes:
        source_width: 원본 가로 픽셀 수.
        source_height: 원본 세로 픽셀 수.
        model_width: 모델 입력 가로 픽셀 수.
        model_height: 모델 입력 세로 픽셀 수.
        scale: 원본 → 모델 배율.
        pad_x: 왼쪽 여백 (모델 픽셀).
        pad_y: 위쪽 여백 (모델 픽셀).
    """

    source_width: int
    source_height: int
    model_width: int
    model_height: int
    scale: float
    pad_x: int
    pad_y: int

    def to_source_box(
        self, ymin: float, xmin: float, ymax: float, xmax: float
    ) -> PixelBox:
        """HailoRT NMS 의 정규화 좌표를 원본 영상 픽셀 박스로 되돌린다.

        좌표는 정수로 반올림(짝수 쪽)하고 원본 영상 안으로 자른다.

        Args:
            ymin: 위쪽 y (0~1, 모델 입력 기준).
            xmin: 왼쪽 x.
            ymax: 아래쪽 y.
            xmax: 오른쪽 x.

        Returns:
            원본 영상 픽셀 좌표의 박스.
        """
        x1 = (xmin * self.model_width - self.pad_x) / self.scale
        y1 = (ymin * self.model_height - self.pad_y) / self.scale
        x2 = (xmax * self.model_width - self.pad_x) / self.scale
        y2 = (ymax * self.model_height - self.pad_y) / self.scale
        right = self.source_width - 1
        bottom = self.source_height - 1
        return PixelBox(
            float(round(min(max(x1, 0.0), right))),
            float(round(min(max(y1, 0.0), bottom))),
            float(round(min(max(x2, 0.0), right))),
            float(round(min(max(y2, 0.0), bottom))),
        )


def gray8_image(frame: Gray8Frame) -> Gray8Image:
    """Gray8Frame 의 픽셀을 (height, width) 배열로 본다.

    stride 가 width 와 같으면 복사하지 않는다 (읽기 전용).

    Args:
        frame: 흑백 프레임.

    Returns:
        (height, width) uint8 배열.
    """
    rows = np.frombuffer(frame.pixels, np.uint8).reshape(frame.height, frame.stride)
    if frame.stride == frame.width:
        return rows
    return np.ascontiguousarray(rows[:, : frame.width])


def _letterbox_rgb(
    gray: Gray8Image, model_width: int, model_height: int
) -> tuple[Gray8Image, LetterboxTransform]:
    height, width = gray.shape
    scale = min(model_width / width, model_height / height)
    resized_width = max(1, int(width * scale))
    resized_height = max(1, int(height * scale))
    pad_x = (model_width - resized_width) // 2
    pad_y = (model_height - resized_height) // 2
    transform = LetterboxTransform(
        width, height, model_width, model_height, scale, pad_x, pad_y
    )
    out = np.full((model_height, model_width, 3), LETTERBOX_PAD_VALUE, np.uint8)
    if (resized_width, resized_height) != (width, height):
        # 실제 크기 변경: Hailo 공식 객체 탐지 예제와 같은 순서
        # (BGR 로 늘려 INTER_CUBIC).
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        resized = cv2.resize(
            bgr, (resized_width, resized_height), interpolation=cv2.INTER_CUBIC
        )
        out[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = resized
        return np.ascontiguousarray(cv2.cvtColor(out, cv2.COLOR_BGR2RGB)), transform
    out[pad_y : pad_y + height, pad_x : pad_x + width] = cv2.cvtColor(
        gray, cv2.COLOR_GRAY2RGB
    )
    return out, transform


def make_model_input(
    gray: Gray8Image, model_width: int, model_height: int, channels: int
) -> tuple[Gray8Image, LetterboxTransform]:
    """흑백 영상으로 HEF 입력(NHWC 의 HWC)을 만든다.

    Args:
        gray: (height, width) 원본 크기 흑백 영상.
        model_width: HEF 입력 가로.
        model_height: HEF 입력 세로.
        channels: HEF 입력 채널 수 (1 또는 3).

    Returns:
        (모델 입력, 좌표 변환). 모델 입력은 (model_height, model_width, channels).

    Raises:
        ValueError: channels 가 1, 3 이 아닐 때.
    """
    height, width = gray.shape
    if channels == 1:
        if (width, height) == (model_width, model_height):
            transform = LetterboxTransform(
                width, height, model_width, model_height, 1.0, 0, 0
            )
            return gray[:, :, None], transform
        rgb, transform = _letterbox_rgb(gray, model_width, model_height)
        return rgb[:, :, :1].copy(), transform
    if channels == 3:
        return _letterbox_rgb(gray, model_width, model_height)
    raise ValueError(f"HEF input must have 1 or 3 channels, got {channels}")
