import os
import unittest
from pathlib import Path

import numpy as np
from argus_vision.double_plateau import (
    DoublePlateauPreprocessor,
    PlateauEma,
    estimate_plateau_factors,
)
from argus_vision.plateau_native import DEFAULT_LIBRARY_PATH, PlateauNativeLibrary
from argus_vision.types import RawFrame

# 기준 구현: 실기체 RPi preprocessor.py 의 estimate_plateau_factors, AdaptivePlateau,
# double_plateau_gray8 (np.percentile, np.median 을 그대로 쓰는 원래 식). 빠른 구현과
# C++ 경로가 이것과 바이트 단위로 같아야 한다.

RAW14_LEVELS = 16384
RAW14_MAX = RAW14_LEVELS - 1


def reference_factors(raw: np.ndarray) -> tuple[float, float]:
    sample = raw[::2, ::2].astype(np.float32)
    p1, p99 = np.percentile(sample, (1.0, 99.0))
    span = max(float(p99 - p1), 1.0)
    span_ratio = float(np.clip(span / RAW14_MAX, 0.0, 1.0))
    horizontal = (
        np.median(np.abs(np.diff(sample, axis=1))) if sample.shape[1] > 1 else 0.0
    )
    vertical = (
        np.median(np.abs(np.diff(sample, axis=0))) if sample.shape[0] > 1 else 0.0
    )
    variation = float((horizontal + vertical) * 0.5)
    variation_ratio = float(np.clip(variation / span, 0.0, 1.0))
    contrast_need = float(np.clip(1.0 - span_ratio / 0.20, 0.0, 1.0))
    noise_score = float(np.clip(variation_ratio / 0.12, 0.0, 1.0))
    upper = float(np.clip(2.0 + 3.0 * contrast_need - 1.25 * noise_score, 1.25, 5.0))
    lower = float(
        np.clip(0.08 + 0.35 * contrast_need * (1.0 - noise_score), 0.05, 0.50)
    )
    return upper, lower


def reference_gray8(raw: np.ndarray, upper: float, lower: float) -> np.ndarray:
    histogram = np.bincount(raw.ravel(), minlength=RAW14_LEVELS).astype(np.float64)
    mean_count = raw.size / RAW14_LEVELS
    modified = np.minimum(histogram, max(1.0, upper * mean_count))
    floor = lower * mean_count
    modified[(histogram > 0.0) & (histogram < floor)] = floor
    cdf = np.cumsum(modified)
    first = cdf[np.flatnonzero(modified)[0]]
    denominator = cdf[-1] - first
    if denominator <= 0.0:
        return np.zeros(raw.shape, np.uint8)
    lut = np.rint(np.clip((cdf - first) * 255.0 / denominator, 0.0, 255.0))
    return lut.astype(np.uint8)[raw]


class ReferenceAdaptive:
    def __init__(self) -> None:
        self.current: tuple[float, float] | None = None

    def process(self, raw: np.ndarray) -> np.ndarray:
        target = reference_factors(raw)
        if self.current is None:
            self.current = target
        else:
            a = 0.15
            u, lo = self.current
            self.current = ((1 - a) * u + a * target[0], (1 - a) * lo + a * target[1])
        return reference_gray8(raw, *self.current)


def make_frame(raw: np.ndarray, frame_id: int = 1, pad: int = 0) -> RawFrame:
    height, width = raw.shape
    stride = width * 2 + pad
    rows = np.zeros((height, stride), np.uint8)
    rows[:, : width * 2] = raw.astype("<u2").view(np.uint8).reshape(height, -1)
    return RawFrame(
        source="ir0",
        session_id="s1",
        frame_id=frame_id,
        unix_ns=10,
        received_monotonic_ns=20,
        width=width,
        height=height,
        stride=stride,
        pixels=rows.tobytes(),
    )


def scenes(rng: np.random.Generator) -> list[np.ndarray]:
    # 실제 열화상과 비슷한 좁은 범위, 넓은 범위, 고른 값, 작은 크기, 홀수 크기,
    # RAW14 보다 큰 값(SDK uint16), 한 값뿐인 영상.
    base = rng.normal(7600, 60, (480, 640))
    base[200:260, 300:380] += 900
    return [
        np.clip(base, 0, RAW14_MAX).astype(np.uint16),
        np.clip(rng.normal(8000, 2500, (480, 640)), 0, RAW14_MAX).astype(np.uint16),
        rng.integers(0, RAW14_LEVELS, (480, 640), dtype=np.uint16),
        rng.integers(7000, 7010, (31, 47), dtype=np.uint16),
        rng.integers(100, 9000, (7, 5), dtype=np.uint16),
        rng.integers(15000, 20000, (64, 80), dtype=np.uint16),
        np.full((48, 64), 5000, np.uint16),
    ]


def native_library_path() -> Path:
    return Path(os.environ.get("ARGUS_PLATEAU_NATIVE_LIB", str(DEFAULT_LIBRARY_PATH)))


class EstimateTest(unittest.TestCase):
    def test_matches_reference_factors(self) -> None:
        rng = np.random.default_rng(7)
        for raw in scenes(rng):
            with self.subTest(shape=raw.shape):
                self.assertEqual(estimate_plateau_factors(raw), reference_factors(raw))


class PlateauEmaTest(unittest.TestCase):
    def test_first_frame_is_used_as_is_then_blends(self) -> None:
        ema = PlateauEma(0.15)
        self.assertEqual(ema.update((3.0, 0.2)), (3.0, 0.2))
        upper, lower = ema.update((4.0, 0.4))
        self.assertAlmostEqual(upper, 3.15)
        self.assertAlmostEqual(lower, 0.23)
        ema.reset()
        self.assertEqual(ema.update((2.0, 0.1)), (2.0, 0.1))

    def test_rejects_alpha_out_of_range(self) -> None:
        with self.assertRaises(ValueError):
            PlateauEma(0.0)


class DoublePlateauPreprocessorTest(unittest.TestCase):
    def check_sequence(self, pre: DoublePlateauPreprocessor) -> None:
        rng = np.random.default_rng(11)
        reference = ReferenceAdaptive()
        base = rng.normal(7600, 80, (480, 640))
        for i in range(12):
            raw = np.clip(
                base + rng.normal(0, 30, base.shape) + i * 40, 0, RAW14_MAX
            ).astype(np.uint16)
            gray = pre.process(make_frame(raw, frame_id=i + 1))
            self.assertEqual(gray.pixels, reference.process(raw).tobytes(), i)

    def test_python_path_matches_reference_over_frames(self) -> None:
        self.check_sequence(DoublePlateauPreprocessor())

    def test_each_scene_matches_reference(self) -> None:
        for raw in scenes(np.random.default_rng(3)):
            with self.subTest(shape=raw.shape):
                pre = DoublePlateauPreprocessor()
                gray = pre.process(make_frame(raw))
                self.assertEqual(
                    gray.pixels, ReferenceAdaptive().process(raw).tobytes()
                )

    def test_keeps_frame_key_size_and_name(self) -> None:
        raw = np.random.default_rng(5).integers(7000, 9000, (6, 8), dtype=np.uint16)
        gray = DoublePlateauPreprocessor().process(make_frame(raw, frame_id=42))
        self.assertEqual(
            (gray.source, gray.session_id, gray.frame_id), ("ir0", "s1", 42)
        )
        self.assertEqual((gray.width, gray.height, gray.stride), (8, 6, 8))
        self.assertEqual((gray.unix_ns, gray.received_monotonic_ns), (10, 20))
        self.assertEqual(gray.preprocess, "double_plateau_adaptive")

    def test_row_padding_does_not_change_output(self) -> None:
        raw = np.random.default_rng(9).integers(7000, 9000, (24, 32), dtype=np.uint16)
        plain = DoublePlateauPreprocessor().process(make_frame(raw))
        padded = DoublePlateauPreprocessor().process(make_frame(raw, pad=6))
        self.assertEqual(plain.pixels, padded.pixels)

    def test_reset_starts_ema_again(self) -> None:
        rng = np.random.default_rng(13)
        first = rng.integers(7000, 7100, (48, 64), dtype=np.uint16)
        second = rng.integers(6000, 9000, (48, 64), dtype=np.uint16)
        pre = DoublePlateauPreprocessor()
        pre.process(make_frame(first, 1))
        pre.reset()
        fresh = DoublePlateauPreprocessor().process(make_frame(second, 1))
        self.assertEqual(pre.process(make_frame(second, 2)).pixels, fresh.pixels)

    @unittest.skipUnless(
        native_library_path().is_file(), "libplateau_native.so 없음 (native/build.sh)"
    )
    def test_native_path_matches_reference(self) -> None:
        library = PlateauNativeLibrary(native_library_path())
        for threads in (1, 4):
            with self.subTest(threads=threads):
                pre = DoublePlateauPreprocessor(library, native_threads=threads)
                self.check_sequence(pre)
                self.assertEqual(pre.native_threads, threads)
                for raw in scenes(np.random.default_rng(3)):
                    pre.reset()
                    gray = pre.process(make_frame(raw))
                    expected = ReferenceAdaptive().process(raw).tobytes()
                    self.assertEqual(gray.pixels, expected, raw.shape)


if __name__ == "__main__":
    unittest.main()
