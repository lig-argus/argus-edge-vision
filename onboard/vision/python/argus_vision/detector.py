# 흑백 프레임에서 객체를 찾는 탐지기 인터페이스.
# 규약: 모델 입력 만들기(채널 수, 크기, 레터박스)와 박스 좌표를
#   원본 영상 픽셀로 되돌리는 일은 구현이 맡는다. 호출하는 쪽은 HEF 의
#   입력 모양을 몰라도 된다. 돌려주는 박스는 원본 영상 안으로 잘라 두고,
#   신뢰도가 경계값 아래인 것은 뺀다.
#   객체가 없으면 빈 목록이다.
# 구현: HailoDetector (HailoRT 비동기 추론과 HEF 안 NMS. 지금 실기체 HEF 는 입력
#   640×480 1채널이라 레터박스가 없고, 640×640 3채널 HEF 는 레터박스를 한다).
#   테스트용 FakeDetector (argus_vision_testing).
# 사용: 탐지 작업자가 프레임마다 detect() 를 부르고, 끝날 때 close() 를 부른다.
#   스레드 하나에서만 부른다.

from abc import ABC, abstractmethod
from collections.abc import Sequence

from argus_vision.types import Detection, Gray8Frame


class IDetector(ABC):
    """흑백 프레임 한 장에서 객체를 찾는다."""

    @property
    @abstractmethod
    def model(self) -> str:
        """추론에 쓰는 모델(HEF) 이름. `perception/observations` 의 model 값."""

    @property
    @abstractmethod
    def labels(self) -> Sequence[str]:
        """class_id 순서의 라벨 목록."""

    @abstractmethod
    def detect(self, frame: Gray8Frame) -> list[Detection]:
        """프레임 한 장을 추론한다.

        Args:
            frame: 전처리가 끝난 원본 크기 흑백 프레임.

        Returns:
            원본 영상 픽셀 좌표의 탐지 목록. 객체가 없으면 빈 목록.

        Raises:
            RuntimeError: NPU 추론이 실패했을 때.
        """

    @abstractmethod
    def close(self) -> None:
        """NPU 와 모델 자원을 놓는다. 두 번 불러도 된다."""
