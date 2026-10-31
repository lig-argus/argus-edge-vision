# RAW14 를 8 비트 흑백으로 바꾸는 전처리 인터페이스.
# 규약: 입력과 같은 크기, 같은 source, session_id, frame_id, 시각을 가진 Gray8Frame 을
#   돌려준다. 박스 좌표계가 바뀌지 않도록 크기 변경, 레터박스, 색 변환은 하지 않는다.
# 구현: DoublePlateauPreprocessor (Double Plateau adaptive. 프레임마다 상한, 하한 계수를
#   EMA 로 갱신하므로 상태가 있다). 테스트용 FakePreprocessor (argus_vision_testing).
# 사용: 탐지 작업자가 프레임마다 process() 를 부르고, 카메라 스트림이 새로 시작될 때
#   reset() 을 부른다.

from abc import ABC, abstractmethod

from argus_vision.types import Gray8Frame, RawFrame


class IPreprocessor(ABC):
    """RAW14 프레임을 원본 크기의 8 비트 흑백 프레임으로 바꾼다."""

    @property
    @abstractmethod
    def name(self) -> str:
        """`vision/frame/gray8` 헤더의 preprocess 값. 예: `double_plateau_adaptive`."""

    @abstractmethod
    def process(self, frame: RawFrame) -> Gray8Frame:
        """프레임 한 장을 전처리한다.

        Args:
            frame: 카메라에서 받은 RAW14 프레임.

        Returns:
            같은 크기, 같은 frame_id 의 흑백 프레임. preprocess 는 name 과 같다.

        Raises:
            ValueError: 지원하지 않는 픽셀 값이나 배치일 때.
        """

    @abstractmethod
    def reset(self) -> None:
        """스트림이 새로 시작될 때 프레임 사이에 이어지는 상태를 지운다."""
