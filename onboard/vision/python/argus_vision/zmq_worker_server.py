# C++ 어댑터(PythonVisionAdapter)의 ipc REQ 를 받아 탐지 작업자를 부르는 REP 서버와
# 실행 진입점.
# 출처: 실기체 RPi `src/argus/perception/detection/python_vision_worker.py`.
# 요청과 응답은 protobuf `argus_msg/perception_ipc.proto` 의 PerceptionRequest,
# PerceptionReply 다 (실기체 RPi 의 MessagePack map `argus.ipc.perception.v1` 과
# 필드 뜻이 같다).
# - ready: 스트림 시작. 전처리 상태를 지우고 모델 이름, 모델 입력 크기를 돌려준다.
# - infer: multipart [요청][RAW14 픽셀]. frame_id 는 늘어나야 하고,
#   시각은 RPi CLOCK_MONOTONIC 이다.
# - shutdown: 응답한 뒤 끝낸다.
# 요청이 틀리거나 처리가 실패하면 ok=False 와 error 를 돌려주고 다음 요청을 기다린다.
# 부모(C++) 가 끝나 stdin 이 닫히면 같이 끝낸다.
# 카메라, 추적, FC, 공개 버스 소켓은 다루지 않는다.

import argparse
import os
import select
import signal
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from types import FrameType

import zmq
from argus_msg import perception_ipc_pb2
from google.protobuf.message import DecodeError

from argus_vision.double_plateau import DoublePlateauPreprocessor
from argus_vision.hailo_detector import (
    HailoDetector,
    parse_class_score_thresholds,
    parse_class_suppress,
    read_labels,
)
from argus_vision.messages import detection_messages
from argus_vision.plateau_native import DEFAULT_LIBRARY_PATH, PlateauNativeLibrary
from argus_vision.preview_sink import IPreviewSink
from argus_vision.types import RAW14_PIXEL_FORMAT, RawFrame
from argus_vision.worker import PythonVisionWorker
from argus_vision.zmq_preview_sink import (
    PreviewHeaderInfo,
    PreviewOptions,
    ZmqPreviewSink,
)

MAX_REQUEST_BYTES = 32 * 1024 * 1024
POLL_MS = 100


@dataclass(frozen=True, slots=True)
class ModelInfo:
    """ready 응답에 넣는 모델 정보.

    Attributes:
        model: 모델(HEF) 이름.
        input_width: 모델 입력 가로.
        input_height: 모델 입력 세로.
    """

    model: str
    input_width: int
    input_height: int


class RequestHandler:
    """요청 한 건을 검사해 작업자를 부르고 응답을 만든다. 소켓은 다루지 않는다."""

    def __init__(
        self,
        worker: PythonVisionWorker,
        model: ModelInfo,
        source: str,
        session_id: str,
    ) -> None:
        """작업자와 세션 정보를 받는다.

        Args:
            worker: 탐지 작업자.
            model: ready 응답의 모델 정보.
            source: 카메라 식별 문자열 (C++ 어댑터가 준 값).
            session_id: 탐지 프로세스 세션 (C++ 어댑터가 준 값).
        """
        self._worker = worker
        self._model = model
        self._source = source
        self._session_id = session_id
        self._last_frame_id: int | None = None
        self.stopping = False

    def handle(self, parts: Sequence[bytes]) -> perception_ipc_pb2.PerceptionReply:
        """요청 한 건을 처리한다. 실패도 응답으로 돌려준다.

        Args:
            parts: multipart 요청.

        Returns:
            응답. 성공이면 ok=True.
        """
        reply = perception_ipc_pb2.PerceptionReply(session_id=self._session_id)
        try:
            request = perception_ipc_pb2.PerceptionRequest()
            try:
                request.ParseFromString(parts[0])
            except DecodeError as exc:
                raise ValueError("request is not a PerceptionRequest") from exc
            if request.session_id != self._session_id:
                raise ValueError("request session mismatch")
            op = request.op
            if op == perception_ipc_pb2.PERCEPTION_OP_READY and len(parts) == 1:
                self._worker.start_stream()
                reply.ok = True
                reply.model = self._model.model
                reply.input_width = self._model.input_width
                reply.input_height = self._model.input_height
            elif op == perception_ipc_pb2.PERCEPTION_OP_SHUTDOWN and len(parts) == 1:
                reply.ok = True
                self.stopping = True
            elif op == perception_ipc_pb2.PERCEPTION_OP_INFER and len(parts) == 2:
                self._infer(request, parts[1], reply)
            else:
                raise ValueError("invalid IPC operation/multipart")
        except Exception as exc:
            reply.ok = False
            reply.error = str(exc)
        return reply

    def _infer(
        self,
        request: perception_ipc_pb2.PerceptionRequest,
        pixels: bytes,
        reply: perception_ipc_pb2.PerceptionReply,
    ) -> None:
        frame_id = request.frame_id
        if self._last_frame_id is not None and frame_id <= self._last_frame_id:
            raise ValueError("frame sequence must advance")
        exposure = (
            request.exposure_monotonic_ns
            if request.HasField("exposure_monotonic_ns")
            else None
        )
        if exposure is not None and not 0 <= exposure <= request.received_monotonic_ns:
            raise ValueError("invalid exposure timestamp")
        if request.pixel_format != RAW14_PIXEL_FORMAT:
            raise ValueError("pixel format must be RAW14_LE16")
        frame = RawFrame(
            source=self._source,
            session_id=self._session_id,
            frame_id=frame_id,
            unix_ns=request.received_unix_ns,
            received_monotonic_ns=request.received_monotonic_ns,
            exposure_monotonic_ns=exposure,
            width=request.width,
            height=request.height,
            stride=request.stride,
            pixels=pixels,
        )
        result = self._worker.handle(frame)
        self._last_frame_id = frame_id
        reply.ok = True
        reply.frame_id = frame_id
        reply.model = self._model.model
        reply.infer_done_monotonic_ns = result.infer_done_monotonic_ns
        reply.preprocess_ms = result.preprocess_ms
        reply.inference_ms = result.inference_ms
        reply.detections.extend(detection_messages(result.detections))


def serve(endpoint: str, handler: RequestHandler, watch_stdin: bool = True) -> None:
    """REP 소켓을 bind 하고 shutdown, 신호, stdin 닫힘까지 요청을 처리한다.

    Args:
        endpoint: bind 주소. 예: `ipc:///tmp/argus-vision-XXXXXX/requests`.
        handler: 요청 처리.
        watch_stdin: True 이면 stdin 이 닫힐 때 끝낸다 (부모 프로세스 종료).
    """
    stopping = False

    def stop(_signum: int, _frame: FrameType | None) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt(zmq.MAXMSGSIZE, MAX_REQUEST_BYTES)
    try:
        socket.bind(endpoint)
        while not stopping and not handler.stopping:
            if watch_stdin and _stdin_closed():
                break
            if not socket.poll(POLL_MS, zmq.POLLIN):
                continue
            parts = socket.recv_multipart()
            socket.send(handler.handle(parts).SerializeToString())
    finally:
        socket.close(linger=0)
        context.term()


def _stdin_closed() -> bool:
    readable = select.select([sys.stdin], [], [], 0)[0]
    return bool(readable) and not os.read(sys.stdin.fileno(), 1)


def _load_native() -> PlateauNativeLibrary | None:
    # ARGUS_NATIVE_PREPROCESS=1 이면 C++ 전처리, 못 불러오면 Python 경로.
    # 환경 변수 이름은 RPi 와 같다.
    if os.environ.get("ARGUS_NATIVE_PREPROCESS") != "1":
        return None
    path = os.environ.get("ARGUS_PLATEAU_NATIVE_LIB", str(DEFAULT_LIBRARY_PATH))
    try:
        return PlateauNativeLibrary(path)
    except OSError as exc:
        print(
            f"preprocess: native C++ unavailable ({exc}); using Python", file=sys.stderr
        )
        return None


def build_parser() -> argparse.ArgumentParser:
    """C++ 어댑터가 넘기는 실행 인자의 parser 를 만든다.

    Returns:
        RPi 의 python_vision_worker 와 같은 인자에 `--publish-gray8`,
        `--gray8-max-fps`, `--class-score-thresholds`, `--class-suppress` 를
        더한 parser.
    """
    p = argparse.ArgumentParser(description="ARGUS 탐지 Python 작업자")
    p.add_argument("--endpoint", required=True)
    p.add_argument("--preview-endpoint", required=True)
    p.add_argument("--session-id", required=True)
    p.add_argument("--source", required=True)
    p.add_argument("--backend", choices=("hailo",), required=True)
    p.add_argument("--hef", required=True)
    p.add_argument("--labels", required=True)
    p.add_argument("--score-threshold", type=float, default=0.7)
    # 실기체는 runtime.env 의 환경 변수로 준다. 인자가 있으면 인자를 쓴다
    p.add_argument(
        "--class-score-thresholds",
        default=os.environ.get("ARGUS_CLASS_SCORE_THRESHOLDS", ""),
        help='클래스별 경계값(라벨 순서). 예: "0.5,0.45,0.35"',
    )
    p.add_argument(
        "--class-suppress",
        default=os.environ.get("ARGUS_CLASS_SUPPRESS", ""),
        help='겹친 다른 클래스 박스 지우기. 예: "tank:car:0.3"',
    )
    p.add_argument(
        "--ir-preprocess",
        choices=("double_plateau_adaptive",),
        default="double_plateau_adaptive",
    )
    p.add_argument("--publish-preview", action="store_true")
    p.add_argument("--publish-npu-input", action="store_true")
    p.add_argument("--publish-gray8", action="store_true")
    p.add_argument("--preview-max-fps", type=float, default=10)
    p.add_argument("--jpeg-quality", type=int, default=75)
    p.add_argument("--npu-input-max-fps", type=float, default=15)
    p.add_argument("--npu-input-jpeg-quality", type=int, default=80)
    p.add_argument("--gray8-max-fps", type=float, default=31)
    return p


def main(argv: Sequence[str] | None = None) -> None:
    """작업자를 조립하고 요청을 처리한다.

    Args:
        argv: 실행 인자. None 이면 sys.argv.
    """
    args = build_parser().parse_args(argv)
    native = _load_native()
    threads = int(os.environ.get("ARGUS_NATIVE_THREADS", "4"))
    preprocessor = DoublePlateauPreprocessor(native, native_threads=threads)
    if native is not None:
        print(f"preprocess: native C++ {native.path}", file=sys.stderr, flush=True)
    labels = read_labels(args.labels)
    detector = HailoDetector(
        args.hef,
        labels,
        args.score_threshold,
        parse_class_score_thresholds(args.class_score_thresholds, labels),
        parse_class_suppress(args.class_suppress, labels),
    )
    preview: IPreviewSink | None = None
    worker: PythonVisionWorker | None = None
    try:
        options = PreviewOptions(
            gray8=args.publish_gray8,
            gray8_max_fps=args.gray8_max_fps,
            overlay=args.publish_preview,
            overlay_max_fps=args.preview_max_fps,
            overlay_jpeg_quality=args.jpeg_quality,
            npu_input=args.publish_npu_input,
            npu_input_max_fps=args.npu_input_max_fps,
            npu_input_jpeg_quality=args.npu_input_jpeg_quality,
        )
        if options.any_enabled:
            info = PreviewHeaderInfo(
                source=args.source,
                session_id=args.session_id,
                backend=args.backend,
                model=detector.model,
                model_width=detector.input_width,
                model_height=detector.input_height,
                model_channels=detector.input_channels,
            )
            preview = ZmqPreviewSink(args.preview_endpoint, info, options)
        worker = PythonVisionWorker(preprocessor, detector, preview)
        handler = RequestHandler(
            worker,
            ModelInfo(detector.model, detector.input_width, detector.input_height),
            args.source,
            args.session_id,
        )
        serve(args.endpoint, handler)
    finally:
        if worker is not None:
            worker.close()
        else:
            if preview is not None:
                preview.close()
            detector.close()
