import unittest

import numpy as np
from argus_vision.model_input import (
    LETTERBOX_PAD_VALUE,
    LetterboxTransform,
    gray8_image,
    make_model_input,
)
from argus_vision.types import Gray8Frame, PixelBox


def make_gray(width: int, height: int, seed: int = 1) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, (height, width), dtype=np.uint8)


class MakeModelInputTest(unittest.TestCase):
    def test_same_size_one_channel_is_not_copied(self) -> None:
        gray = make_gray(640, 480)
        model_input, transform = make_model_input(gray, 640, 480, 1)
        self.assertEqual(model_input.shape, (480, 640, 1))
        self.assertTrue(np.shares_memory(model_input, gray))
        self.assertEqual(transform, LetterboxTransform(640, 480, 640, 480, 1.0, 0, 0))

    def test_square_three_channel_pads_top_and_bottom(self) -> None:
        gray = make_gray(640, 480)
        model_input, transform = make_model_input(gray, 640, 640, 3)
        self.assertEqual(model_input.shape, (640, 640, 3))
        self.assertEqual(
            (transform.scale, transform.pad_x, transform.pad_y), (1.0, 0, 80)
        )
        self.assertTrue((model_input[:80] == LETTERBOX_PAD_VALUE).all())
        self.assertTrue((model_input[560:] == LETTERBOX_PAD_VALUE).all())
        for channel in range(3):
            self.assertTrue((model_input[80:560, :, channel] == gray).all())

    def test_square_one_channel_takes_first_channel(self) -> None:
        gray = make_gray(640, 480)
        model_input, _ = make_model_input(gray, 640, 640, 1)
        self.assertEqual(model_input.shape, (640, 640, 1))
        self.assertTrue((model_input[80:560, :, 0] == gray).all())

    def test_resize_path_scales_and_pads(self) -> None:
        gray = make_gray(320, 200)
        model_input, transform = make_model_input(gray, 640, 640, 3)
        self.assertEqual(model_input.shape, (640, 640, 3))
        self.assertEqual(
            (transform.scale, transform.pad_x, transform.pad_y), (2.0, 0, 120)
        )
        self.assertTrue((model_input[:120] == LETTERBOX_PAD_VALUE).all())

    def test_rejects_other_channel_counts(self) -> None:
        with self.assertRaises(ValueError):
            make_model_input(make_gray(8, 6), 8, 6, 2)


class LetterboxTransformTest(unittest.TestCase):
    def test_maps_back_to_source_pixels(self) -> None:
        transform = LetterboxTransform(640, 480, 640, 640, 1.0, 0, 80)
        box = transform.to_source_box(0.25, 0.5, 0.5, 0.75)
        self.assertEqual(box, PixelBox(320.0, 80.0, 480.0, 240.0))

    def test_clips_to_image_and_rounds_half_to_even(self) -> None:
        transform = LetterboxTransform(640, 480, 640, 480, 1.0, 0, 0)
        box = transform.to_source_box(-0.1, 2.5 / 640, 1.2, 1.1)
        self.assertEqual(box, PixelBox(2.0, 0.0, 639.0, 479.0))


class Gray8ImageTest(unittest.TestCase):
    def test_drops_row_padding(self) -> None:
        frame = Gray8Frame(
            source="ir0",
            session_id="s1",
            frame_id=1,
            unix_ns=0,
            received_monotonic_ns=0,
            width=2,
            height=2,
            stride=3,
            preprocess="fake",
            pixels=bytes([1, 2, 9, 3, 4, 9]),
        )
        self.assertEqual(gray8_image(frame).tolist(), [[1, 2], [3, 4]])


if __name__ == "__main__":
    unittest.main()
