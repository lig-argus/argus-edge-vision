# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""공식 파일/조절 순서·단위 검증. 카메라나 UI는 실행하지 않는다."""

import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/argus/perception/calibration"))
from camera_parameters import read_camera, adjust_camera, camera_text


class CameraParametersTest(unittest.TestCase):
    def seed(self):
        # 합성 nadir 광학 카메라. world xy 접점은 평면 z=0, 높이2m.
        return (
            np.diag([1.0, -1.0, -1.0]),
            np.array([0.0, 0.0, 2.0]),
            np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]),
        )

    def test_official_mapper_format_and_ground_roundtrip(self):
        path = ROOT / "vendor/yaw/ucmc/detector/mapper.py"
        spec = importlib.util.spec_from_file_location("official_mapper", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "cam.txt"
            p.write_text(camera_text(self.seed()))
            mapper = module.Mapper(str(p), "MOT")
            u, v = mapper.xy2uv(1.0, 0.5)
            xy, cov = mapper.uv2xy(np.array([[u], [v]]), np.eye(2))
            np.testing.assert_allclose(xy.flatten(), [1.0, 0.5])
            self.assertTrue(np.linalg.eigvalsh(cov).min() > 0)
            for actual, expected in zip(read_camera(p), self.seed()):
                np.testing.assert_allclose(actual, expected)

    def test_adjustment_order_and_units(self):
        r, t, k = adjust_camera(self.seed(), theta_x=10, theta_y=-5, theta_z=20, focal=30, tz=0.04)
        x, y, z = np.deg2rad([10, -5, 20])
        rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
        ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
        rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
        np.testing.assert_allclose(r, self.seed()[0] @ (rx @ (ry @ rz)))
        self.assertAlmostEqual(t[2], 2.04)
        self.assertEqual(k[0, 0], 530)
        self.assertEqual(k[1, 1], 530)
        self.assertIn("0 0 2040", camera_text((r, t, k)))

    def test_invalid_matrices_and_non_destructive_cli(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "camera.txt"
            original = camera_text(self.seed())
            source.write_text(original)
            cli = ROOT / "src/argus/perception/calibration/camera_parameters.py"
            out = Path(directory) / "new.txt"
            result = subprocess.run(
                [sys.executable, str(cli), "--cam-para", str(source), "--output", str(out)],
                capture_output=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(source.read_text(), original)
            duplicate = subprocess.run(
                [sys.executable, str(cli), "--cam-para", str(source), "--output", str(source)],
                capture_output=True,
            )
            self.assertNotEqual(duplicate.returncode, 0)
            self.assertEqual(source.read_text(), original)
            source.write_text(original.replace("500", "-500"))
            with self.assertRaises(ValueError):
                read_camera(source)
            with self.assertRaises(ValueError):
                adjust_camera(self.seed(), focal=-500)
            with self.assertRaises(ValueError):
                adjust_camera(self.seed(), tz=float("nan"))


if __name__ == "__main__":
    unittest.main()
