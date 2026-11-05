# 미리보기 영상을 C++ 어댑터로 내보내는 끝.
# 규약: offer() 는 기다리지 않고 바로 돌아온다. 보낼 자리가 없으면 그 프레임을 버린다
#   (process_interfaces.md 5.1: 송신 SNDHWM 4). 한 번의 offer() 로 설정에 따라
#   `vision/frame/gray8`, `vision/overlay/jpeg`(박스를 그린 JPEG, 최대 15 FPS),
#   `vision/npu_input/jpeg`(진단용) 중 켜진 것을 내보낸다. 각 FPS 상한은 구현이 지킨다.
# 구현: ZmqPreviewSink (ipc PUSH, 별도 인코딩 스레드). 테스트용 FakePreviewSink
#   (argus_vision_testing).
# 사용: 탐지 작업자가 추론 뒤 프레임마다 offer() 를 부르고, 끝날 때 close() 를 부른다.

from abc import ABC, abstractmethod
from collections.abc import Sequence

from argus_vision.types import Detection, Gray8Frame


class IPreviewSink(ABC):
    """흑백 프레임과 탐지 결과를 미리보기 토픽으로 내보낸다."""

    @abstractmethod
    def offer(self, frame: Gray8Frame, detections: Sequence[Detection]) -> bool:
        """프레임 한 장을 내보내도록 넘긴다.

        Args:
            frame: 전처리가 끝난 흑백 프레임. gray8 토픽은 이 픽셀을 그대로 보낸다.
            detections: 같은 프레임의 탐지 결과. overlay 에 박스로 그린다.

        Returns:
            넘겼으면 True, 자리가 없거나 FPS 상한 때문에 버렸으면 False.
        """

    @abstractmethod
    def close(self) -> None:
        """남은 송신을 멈추고 소켓을 닫는다. 두 번 불러도 된다."""
