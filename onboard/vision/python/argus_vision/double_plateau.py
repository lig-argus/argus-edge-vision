# Double Plateau adaptive 전처리 (RAW14 → 8 비트 흑백).
# 출처: 실기체 RPi `src/argus/perception/preprocessing/` 의
#   `preprocessor.py`(기준 구현)와 `prep_fast.py`(같은 결과의 빠른 구현).
#   팀 GUI 의 "상·하한 Plateau · Adaptive" 와 같은 식이고,
#   학습 데이터도 이 식으로 만들었다.
# 순서:
#   1) 2×2 간격 표본의 p1, p99 와 가로, 세로 차이의 중앙값으로
#      이번 프레임의 상한, 하한 계수를 정한다.
#   2) 계수를 프레임 순서대로 EMA(alpha 0.15)로 갱신한다.
#   3) 히스토그램 상한 자르기, 하한 채우기 → 누적 분포 → LUT 로 8 비트.
# 빠른 구현은 정렬 대신 정수 히스토그램으로 백분위수와 중앙값을 구한다.
#   numpy 1.26 의 np.percentile(linear), np.median 과 연산 순서가 같아
#   결과가 바이트 단위로 같다 (test_double_plateau.py 가 기준 구현과 비교).
# C++ 경로(plateau_native)도 같은 결과다. 쓸 수 없으면 Python 경로를 쓴다.

import numpy as np
import numpy.typing as npt

from argus_vision.arrays import Gray8Image, Raw14Image
from argus_vision.plateau_native import NativeDoublePlateau, PlateauNativeLibrary
from argus_vision.preprocessor import IPreprocessor
from argus_vision.types import RAW14_BYTES_PER_PIXEL, Gray8Frame, RawFrame

RAW14_LEVELS = 16384
RAW14_MAX = RAW14_LEVELS - 1
PLATEAU_UPPER_RANGE = (1.25, 5.0)
PLATEAU_LOWER_RANGE = (0.05, 0.50)
ADAPTIVE_EMA_ALPHA = 0.15


def _clip(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _percentile_linear(
    histogram: npt.NDArray[np.int64], count: int, percents: tuple[float, ...]
) -> npt.NDArray[np.float64]:
    # numpy 1.26 _quantile(method="linear") 를 float32 배열에 쓴 것과 같은 연산.
    quantiles = np.asarray(percents, dtype=np.float64) / 100
    virtual = count * quantiles + (1 + quantiles * (1 - 1 - 1)) - 1
    below = np.clip(np.floor(virtual).astype(np.intp), 0, count - 1)
    above = np.clip(np.floor(virtual).astype(np.intp) + 1, 0, count - 1)
    cumulative = np.cumsum(histogram)
    a = np.searchsorted(cumulative, below, side="right").astype(np.float32)
    b = np.searchsorted(cumulative, above, side="right").astype(np.float32)
    gamma = virtual - np.floor(virtual)
    # numpy 의 _lerp: gamma 가 0.5 이상이면 b 쪽에서 뺀다.
    diff = b - a
    result = a + diff * gamma
    high = gamma >= 0.5
    result[high] = (b - diff * (1 - gamma))[high]
    return result


def _median_of_integers(values: npt.NDArray[np.int32]) -> float:
    # 같은 값의 float32 배열에 np.median 을 쓴 것과 같다: 가운데 값,
    # 짝수 개면 가운데 두 값의 float32 평균.
    count = values.size
    cumulative = np.cumsum(np.bincount(values.ravel()))
    if count % 2:
        return float(np.float32(np.searchsorted(cumulative, count // 2, side="right")))
    middle = np.searchsorted(
        cumulative, np.array([count // 2 - 1, count // 2]), side="right"
    )
    return float(np.mean(middle.astype(np.float32)))


def estimate_plateau_factors(raw: Raw14Image) -> tuple[float, float]:
    """프레임 한 장으로 이번 프레임의 상한, 하한 계수를 정한다.

    Args:
        raw: (height, width) RAW14 영상.

    Returns:
        (상한 계수, 하한 계수).
    """
    sample = raw[::2, ::2]
    p1, p99 = _percentile_linear(np.bincount(sample.ravel()), sample.size, (1.0, 99.0))
    span = max(float(p99 - p1), 1.0)
    span_ratio = _clip(span / RAW14_MAX, 0.0, 1.0)
    signed = sample.astype(np.int32)
    horizontal = (
        _median_of_integers(np.abs(np.diff(signed, axis=1)))
        if signed.shape[1] > 1
        else 0.0
    )
    vertical = (
        _median_of_integers(np.abs(np.diff(signed, axis=0)))
        if signed.shape[0] > 1
        else 0.0
    )
    variation = float(np.float32(np.float32(horizontal) + np.float32(vertical)) * 0.5)
    variation_ratio = _clip(variation / span, 0.0, 1.0)
    contrast_need = _clip(1.0 - span_ratio / 0.20, 0.0, 1.0)
    noise_score = _clip(variation_ratio / 0.12, 0.0, 1.0)
    upper = _clip(2.0 + 3.0 * contrast_need - 1.25 * noise_score, *PLATEAU_UPPER_RANGE)
    lower = _clip(
        0.08 + 0.35 * contrast_need * (1.0 - noise_score), *PLATEAU_LOWER_RANGE
    )
    return upper, lower


def double_plateau_gray8(
    raw: Raw14Image, upper_factor: float, lower_factor: float
) -> Gray8Image:
    """히스토그램 상한 자르기, 하한 채우기 → 누적 분포 → LUT 로 8 비트를 만든다.

    카메라 SDK 가 16383 보다 큰 값을 주면 버리거나 자르지 않고 히스토그램을 늘린다.

    Args:
        raw: (height, width) RAW14 영상.
        upper_factor: 상한 계수 (평균 빈도의 배수).
        lower_factor: 하한 계수 (평균 빈도의 배수).

    Returns:
        같은 크기의 8 비트 영상.
    """
    histogram = np.bincount(raw.ravel(), minlength=RAW14_LEVELS).astype(np.float64)
    mean_count = raw.size / RAW14_LEVELS
    modified = np.minimum(histogram, max(1.0, upper_factor * mean_count))
    lower = lower_factor * mean_count
    modified[(histogram > 0.0) & (histogram < lower)] = lower
    cdf = np.cumsum(modified)
    first = cdf[np.flatnonzero(modified)[0]]
    denominator = cdf[-1] - first
    if denominator <= 0.0:
        return np.zeros(raw.shape, np.uint8)
    lut = np.rint(np.clip((cdf - first) * 255.0 / denominator, 0.0, 255.0)).astype(
        np.uint8
    )
    return np.asarray(lut[raw], dtype=np.uint8)


class PlateauEma:
    """프레임 사이에 이어지는 상한, 하한 계수의 EMA 상태.

    스트림마다 하나를 두고 프레임 순서대로 update() 를 부른다.
    """

    def __init__(self, alpha: float = ADAPTIVE_EMA_ALPHA) -> None:
        """EMA 계수를 정한다.

        Args:
            alpha: 새 프레임 값의 비중 (0 초과 1 이하).

        Raises:
            ValueError: alpha 가 범위 밖일 때.
        """
        if not 0.0 < alpha <= 1.0:
            raise ValueError("adaptive EMA alpha must be in (0, 1]")
        self._alpha = alpha
        self._current: tuple[float, float] | None = None

    def reset(self) -> None:
        """상태를 지운다. 다음 update() 는 그 프레임의 값을 그대로 쓴다."""
        self._current = None

    def update(self, target: tuple[float, float]) -> tuple[float, float]:
        """이번 프레임의 계수를 반영한다.

        Args:
            target: estimate_plateau_factors() 의 결과.

        Returns:
            이번 프레임에 쓸 (상한, 하한) 계수.
        """
        if self._current is None:
            self._current = target
            return target
        a = self._alpha
        old_upper, old_lower = self._current
        self._current = (
            (1 - a) * old_upper + a * target[0],
            (1 - a) * old_lower + a * target[1],
        )
        return self._current


def raw14_view(frame: RawFrame) -> Raw14Image:
    """RawFrame 의 픽셀을 복사 없이 (height, width) uint16 배열로 본다.

    Args:
        frame: RAW14 프레임.

    Returns:
        읽기 전용 배열. stride 가 한 줄보다 길면 연속이 아니다.
    """
    return np.ndarray(
        shape=(frame.height, frame.width),
        dtype="<u2",
        buffer=frame.pixels,
        strides=(frame.stride, RAW14_BYTES_PER_PIXEL),
    )


class DoublePlateauPreprocessor(IPreprocessor):
    """Double Plateau adaptive 전처리. 프레임마다 계수를 EMA 로 갱신하므로 상태가 있다.

    C++ 라이브러리를 넘기면 그 경로를 쓰고, 없으면 Python 경로를 쓴다. 두 경로의
    출력은 같다.
    """

    def __init__(
        self,
        native_library: PlateauNativeLibrary | None = None,
        native_threads: int = 4,
        alpha: float = ADAPTIVE_EMA_ALPHA,
    ) -> None:
        """경로와 EMA 계수를 정한다.

        Args:
            native_library: 불러 둔 C++ 라이브러리. None 이면 Python 경로.
            native_threads: C++ 경로의 OpenMP 스레드 수. 0 이면 OpenMP 기본값.
            alpha: EMA 계수.
        """
        self._library = native_library
        self._threads = native_threads
        self._alpha = alpha
        self._ema = PlateauEma(alpha)
        self._native: NativeDoublePlateau | None = None

    @property
    def name(self) -> str:
        """`double_plateau_adaptive`."""
        return "double_plateau_adaptive"

    @property
    def native_threads(self) -> int | None:
        """C++ 경로의 스레드 수. Python 경로이거나 아직 프레임을 받지 않았으면 None."""
        return None if self._native is None else self._native.threads

    def process(self, frame: RawFrame) -> Gray8Frame:
        """프레임 한 장을 전처리한다.

        Args:
            frame: RAW14 프레임.

        Returns:
            같은 크기, 같은 frame_id 의 흑백 프레임 (stride 는 width).
        """
        raw = raw14_view(frame)
        if self._library is not None:
            gray = self._native_context(frame.width, frame.height).process(raw)
        else:
            factors = self._ema.update(estimate_plateau_factors(raw))
            gray = double_plateau_gray8(raw, *factors)
        return Gray8Frame(
            source=frame.source,
            session_id=frame.session_id,
            frame_id=frame.frame_id,
            unix_ns=frame.unix_ns,
            received_monotonic_ns=frame.received_monotonic_ns,
            width=frame.width,
            height=frame.height,
            stride=frame.width,
            preprocess=self.name,
            pixels=gray.tobytes(),
        )

    def reset(self) -> None:
        """EMA 상태를 지운다 (C++ 경로도 같이)."""
        self._ema.reset()
        if self._native is not None:
            self._native.reset()

    def _native_context(self, width: int, height: int) -> NativeDoublePlateau:
        # 영상 크기가 바뀌면 새 문맥을 만든다 (EMA 상태도 새로 시작).
        native = self._native
        if native is None or (native.width, native.height) != (width, height):
            assert self._library is not None
            if native is not None:
                native.close()
            native = NativeDoublePlateau(
                self._library, width, height, self._threads, self._alpha
            )
            self._native = native
        return native
