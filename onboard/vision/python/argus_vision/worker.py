# 프레임 한 장마다 전처리 → 추론 → 미리보기를 차례로 부르는 탐지 작업자.
# 구현은 시작할 때 넣어 준다 (의존 역전).
# 소켓과 메시지 형식은 zmq_worker_server.py 가 맡는다.
# 출처: 실기체 RPi `src/argus/perception/detection/python_vision_worker.py`
#   의 처리 부분.

import time
from collections.abc import Callable
from dataclasses import dataclass

from argus_vision.detector import IDetector
from argus_vision.preprocessor import IPreprocessor
from argus_vision.preview_sink import IPreviewSink
from argus_vision.types import Detection, Gray8Frame, RawFrame


@dataclass(frozen=True, slots=True)
class FrameResult:
    """프레임 한 장의 탐지 결과와 시간.

    Attributes:
        gray: 전처리한 흑백 프레임.
        detections: 원본 영상 픽셀 좌표의 탐지 목록.
        preprocess_ms: 전처리 시간.
        inference_ms: 모델 입력 만들기부터 NMS 해석까지의 시간.
        infer_done_monotonic_ns: 추론이 끝난 monotonic 시각.
    """

    gray: Gray8Frame
    detections: list[Detection]
    preprocess_ms: float
    inference_ms: float
    infer_done_monotonic_ns: int


class PythonVisionWorker:
    """전처리, 추론, 미리보기 송신의 순서만 맡는다."""

    def __init__(
        self,
        preprocessor: IPreprocessor,
        detector: IDetector,
        preview: IPreviewSink | None = None,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        """구현을 받는다.

        Args:
            preprocessor: RAW14 → 흑백 전처리.
            detector: 탐지기.
            preview: 미리보기 송신. None 이면 보내지 않는다.
            clock_ns: monotonic 시계 (ns). 테스트에서 바꾼다.
        """
        self._preprocessor = preprocessor
        self._detector = detector
        self._preview = preview
        self._clock_ns = clock_ns

    def start_stream(self) -> None:
        """카메라 스트림이 새로 시작될 때 전처리 상태를 지운다."""
        self._preprocessor.reset()

    def handle(self, frame: RawFrame) -> FrameResult:
        """프레임 한 장을 처리한다.

        Args:
            frame: 카메라에서 받은 RAW14 프레임.

        Returns:
            탐지 결과와 시간.

        Raises:
            ValueError: 전처리할 수 없는 프레임일 때.
            RuntimeError: 추론이 실패했거나 미리보기 인코딩 스레드가 실패했을 때.
        """
        t0 = self._clock_ns()
        gray = self._preprocessor.process(frame)
        t1 = self._clock_ns()
        detections = self._detector.detect(gray)
        t2 = self._clock_ns()
        if self._preview is not None:
            self._preview.offer(gray, detections)
        return FrameResult(
            gray=gray,
            detections=detections,
            preprocess_ms=(t1 - t0) / 1e6,
            inference_ms=(t2 - t1) / 1e6,
            infer_done_monotonic_ns=t2,
        )

    def close(self) -> None:
        """미리보기와 탐지기를 닫는다. 하나가 실패해도 나머지를 닫는다.

        Raises:
            RuntimeError: 닫다가 실패한 것이 있을 때 (첫 실패를 원인으로).
        """
        failures: list[BaseException] = []
        for resource in (self._preview, self._detector):
            if resource is None:
                continue
            try:
                resource.close()
            except Exception as exc:
                failures.append(exc)
        if failures:
            raise RuntimeError("worker cleanup failed") from failures[0]
