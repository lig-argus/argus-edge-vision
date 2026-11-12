# 탐지 인터페이스의 테스트용 구현. 카메라, NPU, ZeroMQ 없이 탐지 작업자와 그 사용자를
# 시험할 때 쓴다. 제품 코드는 이 패키지를 import 하지 않는다.

from collections.abc import Sequence

from argus_vision.detector import IDetector
from argus_vision.preprocessor import IPreprocessor
from argus_vision.preview_sink import IPreviewSink
from argus_vision.types import Detection, Gray8Frame, RawFrame


class FakePreprocessor(IPreprocessor):
    """RAW14 의 위 8 비트를 그대로 흑백 값으로 쓰는 전처리.

    받은 프레임과 reset() 횟수를 기록한다.
    """

    def __init__(self) -> None:
        """기록을 비운 상태로 만든다."""
        self.processed: list[RawFrame] = []
        self.reset_count = 0

    @property
    def name(self) -> str:
        """`fake`."""
        return "fake"

    def process(self, frame: RawFrame) -> Gray8Frame:
        """각 픽셀의 위 바이트(리틀 엔디언의 두 번째 바이트)를 흑백 값으로 쓴다.

        Args:
            frame: RAW14 프레임.

        Returns:
            같은 크기, 같은 frame_id 의 흑백 프레임.
        """
        self.processed.append(frame)
        row_bytes = frame.width * 2
        rows = (
            frame.pixels[y * frame.stride : y * frame.stride + row_bytes][1::2]
            for y in range(frame.height)
        )
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
            pixels=b"".join(rows),
        )

    def reset(self) -> None:
        """호출 횟수를 센다."""
        self.reset_count += 1


class FakeDetector(IDetector):
    """미리 정한 탐지 결과를 돌려주는 탐지기.

    받은 프레임을 기록한다. set_detections() 로 다음 결과를,
    set_error() 로 실패를 정한다.
    """

    def __init__(
        self, model: str = "fake-model", labels: Sequence[str] = ("tank",)
    ) -> None:
        """모델 이름과 라벨을 정한다.

        Args:
            model: model 속성이 돌려줄 이름.
            labels: class_id 순서의 라벨.
        """
        self._model = model
        self._labels = tuple(labels)
        self._detections: list[Detection] = []
        self._error: str | None = None
        self.frames: list[Gray8Frame] = []
        self.closed = False

    @property
    def model(self) -> str:
        """생성할 때 정한 모델 이름."""
        return self._model

    @property
    def labels(self) -> Sequence[str]:
        """생성할 때 정한 라벨."""
        return self._labels

    def set_detections(self, detections: Sequence[Detection]) -> None:
        """다음 detect() 부터 돌려줄 결과를 정한다.

        Args:
            detections: 돌려줄 탐지 목록.
        """
        self._detections = list(detections)

    def set_error(self, message: str | None) -> None:
        """다음 detect() 부터 RuntimeError 를 내게 한다. None 이면 끈다.

        Args:
            message: 예외 메시지.
        """
        self._error = message

    def detect(self, frame: Gray8Frame) -> list[Detection]:
        """정해 둔 결과를 돌려준다.

        Args:
            frame: 기록할 프레임.

        Returns:
            set_detections() 로 정한 목록의 복사본.

        Raises:
            RuntimeError: close() 뒤에 부르거나 set_error() 로 실패를 정했을 때.
        """
        if self.closed:
            raise RuntimeError("detector is closed")
        if self._error is not None:
            raise RuntimeError(self._error)
        self.frames.append(frame)
        return list(self._detections)

    def close(self) -> None:
        """닫힘으로 표시한다."""
        self.closed = True


class FakePreviewSink(IPreviewSink):
    """넘겨받은 프레임을 기록하는 미리보기 출력.

    set_accept(False) 로 자리가 없어 버리는 경우를 흉내 낸다.
    """

    def __init__(self) -> None:
        """기록을 비운 상태로 만든다."""
        self._accept = True
        self.offered: list[tuple[Gray8Frame, tuple[Detection, ...]]] = []
        self.dropped = 0
        self.closed = False

    def set_accept(self, accept: bool) -> None:
        """다음 offer() 부터 받을지 버릴지 정한다.

        Args:
            accept: False 이면 버린다.
        """
        self._accept = accept

    def offer(self, frame: Gray8Frame, detections: Sequence[Detection]) -> bool:
        """받거나 버린 것을 기록한다.

        Args:
            frame: 흑백 프레임.
            detections: 같은 프레임의 탐지 결과.

        Returns:
            받았으면 True, 버렸거나 닫혔으면 False.
        """
        if self.closed or not self._accept:
            self.dropped += 1
            return False
        self.offered.append((frame, tuple(detections)))
        return True

    def close(self) -> None:
        """닫힘으로 표시한다."""
        self.closed = True
