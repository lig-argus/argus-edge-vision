# HailoRT 로 YOLOX HEF 를 추론하는 탐지기.
# 출처: 실기체 RPi `src/argus/perception/detection/yolox_detector.py`
#   의 HailoNmsDetector.
# - HailoRT 4.23 비동기 추론(run_async). HEF 안에 NMS 가 들어 있어 출력은 하나이고,
#   클래스별 [ymin, xmin, ymax, xmax, confidence] (모델 입력 기준 정규화 좌표) 목록이다.
# - 다른 프로세스와 NPU 를 나눠 쓰도록 VDevice 그룹 "ARGUS",
#   ROUND_ROBIN 스케줄링을 쓴다.
# - 입력 모양은 HEF 가 정한다. 지금 실기체는 640×480 1채널(레터박스 없음).

import threading
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np

from argus_vision.detector import IDetector
from argus_vision.model_input import LetterboxTransform, gray8_image, make_model_input
from argus_vision.types import Classification, Detection, Gray8Frame

INFER_TIMEOUT_MS = 10_000
VDEVICE_GROUP_ID = "ARGUS"


def parse_nms_output(
    result: Sequence[Sequence[Sequence[float]]],
    transform: LetterboxTransform,
    labels: Sequence[str],
    score_threshold: float,
) -> list[Detection]:
    """HailoRT NMS 출력을 원본 영상 픽셀 좌표의 탐지 목록으로 바꾼다.

    Args:
        result: class_id 순서의 클래스별 행 목록. 행은
            [ymin, xmin, ymax, xmax, confidence] (정규화 좌표).
        transform: 모델 입력을 만들 때의 좌표 변환.
        labels: class_id 순서의 라벨. 모자라면 class_id 를 문자열로 쓴다.
        score_threshold: 이 값보다 낮은 신뢰도는 뺀다.

    Returns:
        신뢰도 내림차순의 탐지 목록.
    """
    detections: list[Detection] = []
    for class_id, rows in enumerate(result):
        for row in rows:
            if len(row) < 5:
                continue
            confidence = float(row[4])
            if confidence < score_threshold:
                continue
            ymin, xmin, ymax, xmax = (float(value) for value in row[:4])
            label = labels[class_id] if class_id < len(labels) else str(class_id)
            detections.append(
                Detection(
                    Classification(class_id, label, confidence),
                    transform.to_source_box(ymin, xmin, ymax, xmax),
                )
            )
    detections.sort(key=lambda item: item.classification.confidence, reverse=True)
    return detections


def read_labels(path: str | Path) -> list[str]:
    """라벨 파일(한 줄에 하나, class_id 순서)을 읽는다. 빈 줄은 건너뛴다.

    Args:
        path: 라벨 파일 경로.

    Returns:
        class_id 순서의 라벨 목록.
    """
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    return [line.strip() for line in lines if line.strip()]


class HailoDetector(IDetector):
    """HailoRT 비동기 추론과 HEF 안 NMS 로 객체를 찾는다."""

    def __init__(
        self, hef_path: str | Path, labels: Sequence[str], score_threshold: float
    ) -> None:
        """HEF 를 불러 NPU 에 올린다.

        Args:
            hef_path: NMS 가 들어 있는 YOLOX HEF.
            labels: class_id 순서의 라벨.
            score_threshold: 신뢰도 경계값. 이보다 낮은 박스는 뺀다.

        Raises:
            FileNotFoundError: HEF 가 없을 때.
            RuntimeError: HailoRT Python 이 없거나, HEF 의 입력이 하나가 아니거나,
                채널이 1, 3 이 아니거나, NMS 출력이 하나가 아닐 때.
        """
        # HailoRT Python 은 RPi 에만 있으므로 쓸 때 import 한다.
        try:
            from hailo_platform import (
                HEF,
                FormatType,
                HailoSchedulingAlgorithm,
                VDevice,
            )
        except ImportError as exc:
            raise RuntimeError(
                "HailoRT Python (hailo_platform) is not installed; install the "
                "HailoRT 4.23 wheel on the RPi"
            ) from exc

        path = Path(hef_path)
        if not path.is_file():
            raise FileNotFoundError(path)
        hef = HEF(str(path))
        input_infos = hef.get_input_vstream_infos()
        if len(input_infos) != 1:
            raise RuntimeError(f"expected one HEF input, got {len(input_infos)}")
        height, width, channels = input_infos[0].shape
        if channels not in (1, 3):
            raise RuntimeError(
                "expected an NHWC input with 1 or 3 channels, "
                f"got {input_infos[0].shape}"
            )

        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        params.group_id = VDEVICE_GROUP_ID
        self._device: Any = VDevice(params)
        self._config_context: Any = None
        self._last_job: Any = None
        try:
            self._infer_model: Any = self._device.create_infer_model(str(path))
            self._infer_model.set_batch_size(1)
            if len(self._infer_model.outputs) != 1:
                raise RuntimeError(
                    "expected one HailoRT NMS output; compile the YOLOX HEF with NMS"
                )
            for output in self._infer_model.outputs:
                output.set_format_type(FormatType.FLOAT32)
            self._config_context = self._infer_model.configure()
            self._configured: Any = self._config_context.__enter__()
        except BaseException:
            self.close()
            raise
        self._model = path.name
        self._labels = tuple(labels)
        self._score_threshold = score_threshold
        self.input_width = int(width)
        self.input_height = int(height)
        self.input_channels = int(channels)

    @property
    def model(self) -> str:
        """HEF 파일 이름."""
        return self._model

    @property
    def labels(self) -> Sequence[str]:
        """class_id 순서의 라벨."""
        return self._labels

    def detect(self, frame: Gray8Frame) -> list[Detection]:
        """프레임 한 장을 추론한다.

        Args:
            frame: 원본 크기 흑백 프레임.

        Returns:
            원본 영상 픽셀 좌표의 탐지 목록 (신뢰도 내림차순).

        Raises:
            RuntimeError: close() 뒤에 불렀거나 NPU 추론이 실패했을 때.
            TimeoutError: 추론이 10 초 안에 끝나지 않았을 때.
        """
        if self._config_context is None:
            raise RuntimeError("detector is closed")
        model_input, transform = make_model_input(
            gray8_image(frame), self.input_width, self.input_height, self.input_channels
        )
        if not model_input.flags.writeable:
            # HailoRT 의 set_buffer 는 쓰기 가능한 배열만 받는다. Gray8Frame 의 bytes 를
            # 그대로 본 배열(레터박스 없는 1채널 HEF)은 읽기 전용이라 복사한다.
            model_input = model_input.copy()
        output = self._infer_model.outputs[0]
        output_buffer = np.empty(output.shape, dtype=np.float32)
        bindings = self._configured.create_bindings(
            output_buffers={output.name: output_buffer}
        )
        bindings.input().set_buffer(model_input)

        done = threading.Event()
        errors: list[BaseException] = []

        def callback(completion_info: object) -> None:
            exception = getattr(completion_info, "exception", None)
            if isinstance(exception, BaseException):
                errors.append(exception)
            done.set()

        self._configured.wait_for_async_ready(timeout_ms=INFER_TIMEOUT_MS)
        self._last_job = self._configured.run_async([bindings], callback)
        if not done.wait(INFER_TIMEOUT_MS / 1000):
            raise TimeoutError("Hailo inference timed out")
        self._last_job.wait(INFER_TIMEOUT_MS)
        if errors:
            raise RuntimeError(f"Hailo inference failed: {errors[0]}")
        result = bindings.output().get_buffer()
        return parse_nms_output(result, transform, self._labels, self._score_threshold)

    def close(self) -> None:
        """추론을 마저 기다리고 NPU 를 놓는다. 두 번 불러도 된다."""
        if self._last_job is not None:
            self._last_job.wait(INFER_TIMEOUT_MS)
            self._last_job = None
        if self._config_context is not None:
            self._config_context.__exit__(None, None, None)
            self._config_context = None
        if self._device is not None:
            self._device.release()
            self._device = None
