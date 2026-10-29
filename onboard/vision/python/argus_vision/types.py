# 탐지 프로세스 안에서 주고받는 값 타입.
# 필드 이름과 뜻은 docs/spec/process_interfaces.md 5.1 (vision/frame/gray8 5.1.5,
# perception/observations 5.1.4)과 C++ perception_api 의 types.hpp 와 같다.
# 시각은 RPi monotonic clock 의 ns, 박스 좌표는 원본 영상 픽셀
# (원점 왼쪽 위, y 아래 방향)이다.
# 픽셀은 bytes 로 두어 이 모듈이 numpy, OpenCV, HailoRT 에 의존하지 않게 한다. 구현은
# numpy.frombuffer 로 복사 없이 배열로 본다.

from dataclasses import dataclass

RAW14_PIXEL_FORMAT = "RAW14_LE16"
RAW14_BYTES_PER_PIXEL = 2
GRAY8_ENCODING = "gray8"


def _check_layout(
    width: int, height: int, stride: int, min_stride: int, pixel_bytes: int
) -> None:
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if stride < min_stride:
        raise ValueError("stride is smaller than one row")
    if pixel_bytes != stride * height:
        raise ValueError("pixels must be stride * height bytes")


@dataclass(frozen=True, slots=True, kw_only=True)
class RawFrame:
    """카메라에서 받은 RAW14 프레임 한 장.

    C++ 카메라 수신부가 frame_id 와 수신 시각을 붙여 Python 작업자에 넘긴다.
    픽셀은 리틀 엔디언 16 비트(`RAW14_LE16`)이며 값 범위는 0~16383 이다.

    Attributes:
        source: 카메라 식별 문자열.
        session_id: 탐지 프로세스 실행마다 바뀌는 값.
        frame_id: 카메라 프레임 번호. 탐지 결과와 같은 값이다.
        unix_ns: RPi 수신 시각 (wall clock).
        received_monotonic_ns: RPi 수신 시각 (monotonic). 노출 시각이 아니다.
        exposure_monotonic_ns: 센서 노출 시각. 지금 카메라는 제공하지 않아 None.
        width: 가로 픽셀 수.
        height: 세로 픽셀 수.
        stride: 한 줄의 바이트 수 (width * 2 이상).
        pixels: stride * height 바이트.
    """

    source: str
    session_id: str
    frame_id: int
    unix_ns: int
    received_monotonic_ns: int
    exposure_monotonic_ns: int | None = None
    width: int
    height: int
    stride: int
    pixels: bytes

    def __post_init__(self) -> None:
        """픽셀 배치가 맞는지 확인한다.

        Raises:
            ValueError: 크기가 0 이하이거나, stride 가 한 줄보다 작거나,
                픽셀 바이트 수가 stride * height 와 다를 때.
        """
        _check_layout(
            self.width,
            self.height,
            self.stride,
            self.width * RAW14_BYTES_PER_PIXEL,
            len(self.pixels),
        )


@dataclass(frozen=True, slots=True, kw_only=True)
class Gray8Frame:
    """전처리가 끝난 8 비트 흑백 프레임 (`vision/frame/gray8` 헤더와 픽셀).

    레터박스 전 원본 크기이며 박스 좌표계와 같다.

    Attributes:
        source: 카메라 식별 문자열.
        session_id: 탐지 프로세스 실행마다 바뀌는 값.
        frame_id: RawFrame 과 같은 값.
        unix_ns: RPi 수신 시각 (wall clock).
        received_monotonic_ns: RPi 수신 시각 (monotonic).
        width: 가로 픽셀 수.
        height: 세로 픽셀 수.
        stride: 한 줄의 바이트 수 (width 이상).
        preprocess: 전처리 이름. 예: `double_plateau_adaptive`.
        pixels: stride * height 바이트.
    """

    source: str
    session_id: str
    frame_id: int
    unix_ns: int
    received_monotonic_ns: int
    width: int
    height: int
    stride: int
    preprocess: str
    pixels: bytes

    def __post_init__(self) -> None:
        """픽셀 배치가 맞는지 확인한다.

        Raises:
            ValueError: 크기가 0 이하이거나, stride 가 한 줄보다 작거나,
                픽셀 바이트 수가 stride * height 와 다를 때.
        """
        _check_layout(
            self.width, self.height, self.stride, self.width, len(self.pixels)
        )


@dataclass(frozen=True, slots=True)
class PixelBox:
    """원본 영상 픽셀 좌표의 박스. (x1, y1) 왼쪽 위, (x2, y2) 오른쪽 아래.

    Attributes:
        x1: 왼쪽 x.
        y1: 위쪽 y.
        x2: 오른쪽 x.
        y2: 아래쪽 y.
    """

    x1: float
    y1: float
    x2: float
    y2: float

    def __post_init__(self) -> None:
        """좌표 순서를 확인한다.

        Raises:
            ValueError: x2 < x1 또는 y2 < y1 일 때.
        """
        if self.x2 < self.x1 or self.y2 < self.y1:
            raise ValueError("box corners are reversed")

    @property
    def width(self) -> float:
        """박스 너비 (픽셀)."""
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        """박스 높이 (픽셀)."""
        return self.y2 - self.y1


@dataclass(frozen=True, slots=True)
class Classification:
    """분류 결과. class_id 와 label 의 대응은 모델마다 다르다.

    Attributes:
        class_id: 모델의 클래스 번호.
        label: 클래스 이름.
        confidence: 신뢰도 (0~1).
    """

    class_id: int
    label: str
    confidence: float

    def __post_init__(self) -> None:
        """신뢰도 범위를 확인한다.

        Raises:
            ValueError: confidence 가 0~1 밖일 때.
        """
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be within 0..1")


@dataclass(frozen=True, slots=True)
class Detection:
    """탐지 한 건 (`perception/observations` 의 detections[] 원소).

    track_id 는 없다. 추적기가 붙인다.

    Attributes:
        classification: 클래스와 신뢰도.
        box: 원본 영상 픽셀 좌표의 박스.
    """

    classification: Classification
    box: PixelBox
