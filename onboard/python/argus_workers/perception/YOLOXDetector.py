from __future__ import annotations

import threading
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Protocol

import numpy as np

from .Detection import Detection
from .Preprocessor import LetterboxTransform


class Detector(Protocol):
    model_name: str
    input_width: int
    input_height: int

    def infer(
        self,
        model_input: np.ndarray,
        transform: LetterboxTransform,
        timing: dict[str, int] | None = None,
    ) -> list[Detection]: ...
    def close(self) -> None: ...


class MockDetector:
    """Hardware-free backend for testing the camera and bus."""

    model_name = "mock-yolox"
    input_width = 640
    input_height = 640

    def __init__(self, labels: Sequence[str]):
        self._labels = labels or ["target"]
        self._frame = 0

    def infer(
        self,
        _model_input: np.ndarray,
        transform: LetterboxTransform,
        timing: dict[str, int] | None = None,
    ) -> list[Detection]:
        width, height = transform.source_width, transform.source_height
        box_width, box_height = max(40, width // 5), max(40, height // 5)
        travel = max(1, width - box_width)
        x1 = (self._frame * 7) % travel
        y1 = max(0, height // 2 - box_height // 2)
        self._frame += 1
        if timing is not None:
            timing["input_copy_ns"] = 0
            timing["hw_wait_ns"] = 0
            timing["output_fetch_ns"] = 0
            timing["nms_parse_ns"] = 0
        return [Detection(0, self._labels[0], 0.90, x1, y1, x1 + box_width, y1 + box_height)]

    def close(self) -> None:
        pass


class HailoNmsDetector:
    """HailoRT 4.23 detector for a HEF containing HailoRT NMS postprocess."""

    def __init__(self, hef_path: str, labels: Sequence[str], score_threshold: float = 0.35):
        try:
            from hailo_platform import HEF, FormatType, HailoSchedulingAlgorithm, VDevice
        except ImportError as exc:
            raise RuntimeError(
                "PyHailoRT is not installed. Install the HailoRT 4.23 Python wheel on the Pi."
            ) from exc

        path = Path(hef_path)
        if not path.is_file():
            raise FileNotFoundError(path)

        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        params.group_id = "ARGUS"
        self._device = VDevice(params)
        self._hef = HEF(str(path))
        self._infer_model = self._device.create_infer_model(str(path))
        self._infer_model.set_batch_size(1)
        for output in self._infer_model.outputs:
            output.set_format_type(FormatType.FLOAT32)
        self._config_context = self._infer_model.configure()
        self._configured_model = self._config_context.__enter__()
        self._last_job = None
        self._labels = list(labels)
        self._score_threshold = score_threshold
        self.model_name = path.name

        input_infos = self._hef.get_input_vstream_infos()
        if len(input_infos) != 1:
            raise RuntimeError(f"expected one HEF input, got {len(input_infos)}")
        self.input_height, self.input_width, channels = input_infos[0].shape
        if channels != 3:
            raise RuntimeError(f"expected an NHWC RGB input with 3 channels, got {input_infos[0].shape}")
        if len(self._infer_model.outputs) != 1:
            raise RuntimeError(
                "this runtime expects one HailoRT NMS output; compile the YOLOX HEF with NMS enabled"
            )

    def infer(
        self,
        model_input: np.ndarray,
        transform: LetterboxTransform,
        timing: dict[str, int] | None = None,
    ) -> list[Detection]:
        t0 = time.perf_counter_ns() if timing is not None else 0
        output = self._infer_model.outputs[0]
        output_name = output.name
        output_buffer = np.empty(output.shape, dtype=np.float32)
        bindings = self._configured_model.create_bindings(output_buffers={output_name: output_buffer})
        bindings.input().set_buffer(model_input)
        t1 = time.perf_counter_ns() if timing is not None else 0

        done = threading.Event()
        error: list[BaseException] = []

        def callback(completion_info: object) -> None:
            exception = getattr(completion_info, "exception", None)
            if exception is not None:
                error.append(exception)
            done.set()

        self._configured_model.wait_for_async_ready(timeout_ms=10_000)
        self._last_job = self._configured_model.run_async([bindings], callback)
        if not done.wait(10.0):
            raise TimeoutError("Hailo inference timed out")
        self._last_job.wait(10_000)
        if error:
            raise RuntimeError(f"Hailo inference failed: {error[0]}")
        t2 = time.perf_counter_ns() if timing is not None else 0

        result = bindings.output().get_buffer()
        t3 = time.perf_counter_ns() if timing is not None else 0

        detections = self._parse_nms(result, transform)

        if timing is not None:
            t4 = time.perf_counter_ns()
            timing["input_copy_ns"] = t1 - t0
            timing["hw_wait_ns"] = t2 - t1
            timing["output_fetch_ns"] = t3 - t2
            timing["nms_parse_ns"] = t4 - t3

        return detections

    def _parse_nms(self, result: object, transform: LetterboxTransform) -> list[Detection]:
        detections: list[Detection] = []
        # HailoRT NMS output is grouped by class. Each row is
        # [ymin, xmin, ymax, xmax, confidence] in normalized coordinates.
        for class_id, class_detections in enumerate(result):
            for row in class_detections:
                if len(row) < 5:
                    continue
                confidence = float(row[4])
                if confidence < self._score_threshold:
                    continue
                x1, y1, x2, y2 = transform.model_normalized_yxyx_to_source_xyxy(row[:4])
                label = self._labels[class_id] if class_id < len(self._labels) else str(class_id)
                detections.append(
                    Detection(class_id, label, confidence, x1, y1, x2, y2)
                )
        detections.sort(key=lambda item: item.confidence, reverse=True)
        return detections

    def close(self) -> None:
        if self._last_job is not None:
            self._last_job.wait(10_000)
        if self._config_context is not None:
            self._config_context.__exit__(None, None, None)
            self._config_context = None
        if self._device is not None:
            self._device.release()
            self._device = None


class HailoNmsDetectorSync(HailoNmsDetector):
    """Same setup as HailoNmsDetector, but uses ConfiguredInferModel.run()
    (blocking, synchronous) instead of run_async() + threading.Event.

    Latency-experiment variant only: isolates whether the gap between
    hw_wait_ms and HailoRT's own reported HW latency comes from the async
    callback/Event plumbing, or from GIL/OS-scheduling overhead inherent to
    any Python<->HailoRT call. Select with --backend hailo-sync.
    """

    def infer(
        self,
        model_input: np.ndarray,
        transform: LetterboxTransform,
        timing: dict[str, int] | None = None,
    ) -> list[Detection]:
        t0 = time.perf_counter_ns() if timing is not None else 0
        output = self._infer_model.outputs[0]
        output_name = output.name
        output_buffer = np.empty(output.shape, dtype=np.float32)
        bindings = self._configured_model.create_bindings(output_buffers={output_name: output_buffer})
        bindings.input().set_buffer(model_input)
        t1 = time.perf_counter_ns() if timing is not None else 0

        self._configured_model.run([bindings], 10_000)
        t2 = time.perf_counter_ns() if timing is not None else 0

        result = bindings.output().get_buffer()
        t3 = time.perf_counter_ns() if timing is not None else 0

        detections = self._parse_nms(result, transform)

        if timing is not None:
            t4 = time.perf_counter_ns()
            timing["input_copy_ns"] = t1 - t0
            timing["hw_wait_ns"] = t2 - t1
            timing["output_fetch_ns"] = t3 - t2
            timing["nms_parse_ns"] = t4 - t3

        return detections
