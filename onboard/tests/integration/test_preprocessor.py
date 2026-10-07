import numpy as np
import pytest
from argus_workers.perception.Preprocessor import decode_source_frame, letterbox_bgr_to_rgb


@pytest.mark.parametrize("mode", ["minmax", "fixed14"])
def test_raw14_existing_formula_and_letterbox(mode):
    raw = np.array([[0, 2000, 8192], [16383, 4000, 10000]], dtype="<u2")
    header = dict(width=3, height=2, stride=6, pixel_format="RAW14_LE16")
    bgr = decode_source_frame(header, raw.tobytes(), mode)
    scale = 255.0 / (16383 if mode == "fixed14" else int(raw.max()) - int(raw.min()))
    expected = np.rint(raw.astype(np.float32) * scale).astype(np.uint8)
    assert np.array_equal(bgr[:, :, 0], expected)
    assert np.array_equal(bgr[:, :, 0], bgr[:, :, 1]) and np.array_equal(bgr[:, :, 1], bgr[:, :, 2])
    rgb, transform = letterbox_bgr_to_rgb(bgr, 640, 640)
    assert rgb.shape == (640, 640, 3) and rgb.dtype == np.uint8 and rgb.flags.c_contiguous
    assert transform.source_width == 3 and transform.source_height == 2


def test_raw14_out_of_range_and_wrong_payload_rejected():
    header = dict(width=1, height=1, stride=2, pixel_format="RAW14_LE16")
    with pytest.raises(ValueError, match="masking"):
        decode_source_frame(header, np.array([65535], dtype="<u2").tobytes(), "minmax")
    with pytest.raises(ValueError, match="layout"):
        decode_source_frame(header, b"", "minmax")


def test_flat_ir_image_is_valid_black_frame():
    raw = np.full((3, 2), 7000, dtype="<u2")
    header = dict(width=2, height=3, stride=4, pixel_format="RAW14_LE16")
    assert not decode_source_frame(header, raw.tobytes(), "minmax").any()
