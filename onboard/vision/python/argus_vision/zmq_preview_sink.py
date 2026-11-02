# 미리보기 영상을 ipc PUSH 로 C++ 어댑터에 보내는 IPreviewSink 구현.
# 출처: 실기체 RPi `src/argus/perception/preview/overlay_encoder.py` 의 OverlayEncoder.
# - offer() 는 큐(1칸)에 넣고 바로 돌아온다. 큐가 차 있으면 오래된 것을 버리고
#   새 것을 넣는다. JPEG 인코딩과 송신은 별도 스레드가 한다.
#   추론 버퍼에는 그리지 않는다.
# - 토픽마다 FPS 상한을 지킨다
#   (평균 속도를 넘지 않게 하되 도착 흔들림 한 프레임은 허용).
# - 송신 SNDHWM 4: 한 프레임에 최대 3개(gray8, overlay, npu_input)가 연달아 나가므로
#   최소 3에 여유 1칸 (process_interfaces.md 5.1). 막히면 그 메시지를 버린다.
# - `vision/frame/gray8` 헤더는 process_interfaces.md 5.1.5, overlay 와 npu_input 헤더는
#   RPi 코드와 같다.

import math
import queue
import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import zmq

from argus_vision.messagepack import pack_message
from argus_vision.model_input import gray8_image, make_model_input
from argus_vision.preview_sink import IPreviewSink
from argus_vision.types import GRAY8_ENCODING, Detection, Gray8Frame

PREVIEW_SNDHWM = 4
GRAY8_TOPIC = "vision/frame/gray8"
OVERLAY_TOPIC = "vision/overlay/jpeg"
NPU_INPUT_TOPIC = "vision/npu_input/jpeg"
_BOX_COLOR = (0, 255, 0)


@dataclass(frozen=True, slots=True, kw_only=True)
class PreviewOptions:
    """내보낼 토픽과 각각의 FPS 상한, JPEG 품질.

    Attributes:
        gray8: `vision/frame/gray8` 를 보낼지. 기본 꺼짐 (5.1).
        gray8_max_fps: gray8 FPS 상한.
        overlay: `vision/overlay/jpeg` 를 보낼지.
        overlay_max_fps: overlay FPS 상한 (5.1: 최대 15).
        overlay_jpeg_quality: overlay JPEG 품질.
        npu_input: `vision/npu_input/jpeg` 를 보낼지.
        npu_input_max_fps: npu_input FPS 상한.
        npu_input_jpeg_quality: npu_input JPEG 품질.
    """

    gray8: bool = False
    gray8_max_fps: float = 30.0
    overlay: bool = False
    overlay_max_fps: float = 10.0
    overlay_jpeg_quality: int = 75
    npu_input: bool = False
    npu_input_max_fps: float = 15.0
    npu_input_jpeg_quality: int = 80

    @property
    def any_enabled(self) -> bool:
        """켜진 토픽이 하나라도 있으면 True."""
        return self.gray8 or self.overlay or self.npu_input


@dataclass(frozen=True, slots=True, kw_only=True)
class PreviewHeaderInfo:
    """미리보기 헤더에 넣는 탐지 프로세스 정보.

    Attributes:
        source: 카메라 식별 문자열. C++ 어댑터가 자기 값과 같은지 확인한다.
        session_id: 탐지 프로세스의 세션. C++ 어댑터가 자기 값과 같은지 확인한다.
        backend: 추론 방식 이름. 예: `hailo`.
        model: 모델(HEF) 이름.
        model_width: 모델 입력 가로 (npu_input 영상을 만들 때).
        model_height: 모델 입력 세로.
        model_channels: 모델 입력 채널 수.
    """

    source: str
    session_id: str
    backend: str
    model: str
    model_width: int
    model_height: int
    model_channels: int


class FpsPacer:
    """평균 속도를 FPS 상한 아래로 두되 도착 흔들림 한 프레임은 허용한다."""

    def __init__(self, max_fps: float) -> None:
        """FPS 상한을 정한다.

        Args:
            max_fps: 초당 최대 프레임 수.

        Raises:
            ValueError: 유한한 양수가 아닐 때.
        """
        if not math.isfinite(max_fps) or max_fps <= 0:
            raise ValueError("preview FPS must be finite and positive")
        self._fps = max_fps
        self._credit = 2.0
        self._last: float | None = None

    def due(self, now: float) -> bool:
        """지금 프레임을 보내도 되는지 정하고, 보내면 몫을 하나 쓴다.

        Args:
            now: monotonic 시각 (초).

        Returns:
            보내도 되면 True.
        """
        if self._last is not None:
            elapsed = max(0.0, now - self._last)
            self._credit = min(2.0, self._credit + elapsed * self._fps)
        self._last = now
        if self._credit + 1e-9 < 1.0:
            return False
        self._credit = max(0.0, self._credit - 1.0)
        return True


@dataclass(frozen=True, slots=True)
class _Job:
    frame: Gray8Frame
    detections: tuple[Detection, ...]
    gray8: bool
    overlay: bool
    npu_input: bool


_STOP = object()


class ZmqPreviewSink(IPreviewSink):
    """ipc PUSH 로 미리보기 영상을 C++ 어댑터에 보낸다 (별도 인코딩 스레드)."""

    def __init__(
        self,
        endpoint: str,
        info: PreviewHeaderInfo,
        options: PreviewOptions,
        clock: Any = time.monotonic,
    ) -> None:
        """인코딩 스레드를 시작한다. 스레드가 PUSH 소켓을 만들어 endpoint 에 연결한다.

        Args:
            endpoint: C++ 어댑터의 미리보기 PULL 주소. 예: `ipc:///tmp/.../preview`.
            info: 헤더에 넣을 탐지 프로세스 정보.
            options: 켤 토픽과 FPS 상한, JPEG 품질.
            clock: FPS 상한에 쓰는 monotonic 시계 (초). 테스트에서 바꾼다.
        """
        self._endpoint = endpoint
        self._info = info
        self._options = options
        self._clock = clock
        self._gray8_pacer = FpsPacer(options.gray8_max_fps)
        self._overlay_pacer = FpsPacer(options.overlay_max_fps)
        self._npu_pacer = FpsPacer(options.npu_input_max_fps)
        self._queue: queue.Queue[_Job | object] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._error: BaseException | None = None
        self._closed = False
        self.published = 0
        self.dropped = 0
        self._thread = threading.Thread(target=self._run, name="ZmqPreviewSink")
        self._thread.start()

    def offer(self, frame: Gray8Frame, detections: Sequence[Detection]) -> bool:
        """프레임 한 장을 인코딩 스레드에 넘긴다. 기다리지 않는다.

        Args:
            frame: 원본 크기 흑백 프레임.
            detections: 같은 프레임의 탐지 결과.

        Returns:
            넘겼으면 True. 켜진 토픽이 모두 FPS 상한에 걸렸거나 닫혔으면 False.

        Raises:
            RuntimeError: 인코딩 스레드가 실패했을 때.
        """
        if self._error is not None:
            raise RuntimeError("preview encoder failed") from self._error
        if self._closed:
            return False
        now = self._clock()
        options = self._options
        job = _Job(
            frame,
            tuple(detections),
            gray8=options.gray8 and self._gray8_pacer.due(now),
            overlay=options.overlay and self._overlay_pacer.due(now),
            npu_input=options.npu_input and self._npu_pacer.due(now),
        )
        if not (job.gray8 or job.overlay or job.npu_input):
            return False
        try:
            self._queue.put_nowait(job)
        except queue.Full:
            # 인코딩이 밀리면 오래된 프레임을 버린다.
            try:
                self._queue.get_nowait()
                self.dropped += 1
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(job)
            except queue.Full:
                self.dropped += 1
                return False
        return True

    def close(self) -> None:
        """인코딩 스레드를 멈추고 소켓을 닫는다. 두 번 불러도 된다.

        Raises:
            RuntimeError: 스레드가 3 초 안에 멈추지 않았거나 실패했을 때.
        """
        if not self._closed:
            self._closed = True
            self._stop.set()
            try:
                self._queue.put_nowait(_STOP)
            except queue.Full:
                pass
            self._thread.join(timeout=3)
        if self._thread.is_alive():
            raise RuntimeError("preview encoder did not stop")
        if self._error is not None:
            raise RuntimeError("preview encoder failed") from self._error

    def _run(self) -> None:
        context = zmq.Context()
        socket = context.socket(zmq.PUSH)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.SNDHWM, PREVIEW_SNDHWM)
        socket.setsockopt(zmq.IMMEDIATE, 1)
        try:
            socket.connect(self._endpoint)
            while not self._stop.is_set():
                try:
                    job = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if not isinstance(job, _Job):
                    break
                self._encode(socket, job)
        except BaseException as exc:
            self._error = exc
        finally:
            socket.close(linger=0)
            context.term()

    def _encode(self, socket: Any, job: _Job) -> None:
        frame = job.frame
        info = self._info
        common: dict[str, Any] = {
            "source": info.source,
            "session_id": info.session_id,
            "frame_id": frame.frame_id,
            "seq": frame.frame_id,
        }
        if job.gray8:
            gray8_header = dict(
                common,
                schema="argus.vision.frame.v1",
                unix_ns=frame.unix_ns,
                received_monotonic_ns=frame.received_monotonic_ns,
                width=frame.width,
                height=frame.height,
                stride=frame.stride,
                encoding=GRAY8_ENCODING,
                preprocess=frame.preprocess,
            )
            self._send(socket, GRAY8_TOPIC, gray8_header, frame.pixels)
        if not (job.overlay or job.npu_input):
            return
        gray = gray8_image(frame)
        jpeg_common = dict(
            common,
            encoding="jpeg",
            backend=info.backend,
            model=info.model,
            preprocess=frame.preprocess,
            detections=[
                {
                    "label": d.classification.label,
                    "confidence": float(d.classification.confidence),
                    "x1": int(d.box.x1),
                    "y1": int(d.box.y1),
                    "x2": int(d.box.x2),
                    "y2": int(d.box.y2),
                }
                for d in job.detections
            ],
        )
        if job.overlay:
            image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            for d in job.detections:
                x1, y1 = int(d.box.x1), int(d.box.y1)
                x2, y2 = int(d.box.x2), int(d.box.y2)
                cv2.rectangle(image, (x1, y1), (x2, y2), _BOX_COLOR, 2)
                cv2.putText(
                    image,
                    f"{d.classification.label} {d.classification.confidence:.2f}",
                    (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    _BOX_COLOR,
                    2,
                    cv2.LINE_AA,
                )
            overlay_header = dict(
                jpeg_common,
                schema="argus.vision.overlay.v1",
                unix_ns=frame.unix_ns,
                width=frame.width,
                height=frame.height,
                received_monotonic_ns=frame.received_monotonic_ns,
            )
            jpeg = _encode_jpeg(image, self._options.overlay_jpeg_quality)
            self._send(socket, OVERLAY_TOPIC, overlay_header, jpeg)
        if job.npu_input:
            model_input, transform = make_model_input(
                gray, info.model_width, info.model_height, info.model_channels
            )
            npu_header = dict(
                jpeg_common,
                schema="argus.vision.npu_input.v1",
                t_capture_us=frame.received_monotonic_ns // 1000,
                src_w=transform.source_width,
                src_h=transform.source_height,
                lb_scale=transform.scale,
                lb_pad_x=transform.pad_x,
                lb_pad_y=transform.pad_y,
                preprocess="letterbox_bgr_to_rgb",
            )
            if model_input.shape[2] == 1:
                npu_image = model_input[:, :, 0]
            else:
                npu_image = cv2.cvtColor(model_input, cv2.COLOR_RGB2BGR)
            jpeg = _encode_jpeg(npu_image, self._options.npu_input_jpeg_quality)
            self._send(socket, NPU_INPUT_TOPIC, npu_header, jpeg)

    def _send(
        self, socket: Any, topic: str, header: dict[str, Any], payload: bytes
    ) -> None:
        try:
            socket.send_multipart(
                [topic.encode(), pack_message(header), payload], flags=zmq.NOBLOCK
            )
            self.published += 1
        except zmq.Again:
            self.dropped += 1


def _encode_jpeg(image: Any, quality: int) -> bytes:
    # baseline JPEG (progressive, optimize 끔). OpenCV 버전이 바뀌어도 같은 형식.
    ok, encoded = cv2.imencode(
        ".jpg",
        np.ascontiguousarray(image),
        [
            cv2.IMWRITE_JPEG_QUALITY,
            quality,
            cv2.IMWRITE_JPEG_PROGRESSIVE,
            0,
            cv2.IMWRITE_JPEG_OPTIMIZE,
            0,
        ],
    )
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return bytes(encoded.tobytes())
