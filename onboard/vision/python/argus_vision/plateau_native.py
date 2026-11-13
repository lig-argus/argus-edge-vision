# Double Plateau adaptive 의 C++ 라이브러리(native/plateau_native.cpp)를
# ctypes 로 부른다.
# 출력은 Python 경로(double_plateau.py)와 바이트 단위로 같다. 실기체 RPi 에서
# 프레임당 7.4 → 1.6 ms (OpenMP 4스레드).
# 라이브러리는 onboard CMake 타깃 argus_plateau_native 다
# (onboard/vision/CMakeLists.txt. native/build.sh 가 그 타깃만 빌드).
# 기본 경로는 그 출력이다.

import ctypes
import os
from pathlib import Path

import numpy as np
import numpy.typing as npt

# 저장소 루트의 build/onboard (README "onboard 구조와 빌드")
DEFAULT_LIBRARY_PATH = (
    Path(__file__).resolve().parents[4]
    / "build"
    / "onboard"
    / "vision"
    / "libplateau_native.so"
)


class PlateauNativeLibrary:
    """불러 온 libplateau_native.so 와 함수 형식."""

    def __init__(self, path: str | os.PathLike[str] = DEFAULT_LIBRARY_PATH) -> None:
        """라이브러리를 불러 함수 형식을 정한다.

        Args:
            path: libplateau_native.so 경로.

        Raises:
            OSError: 파일이 없거나 불러올 수 없을 때.
        """
        lib = ctypes.CDLL(os.fspath(path))
        lib.pn_create.restype = ctypes.c_void_p
        lib.pn_create.argtypes = [
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_double,
        ]
        lib.pn_destroy.restype = None
        lib.pn_destroy.argtypes = [ctypes.c_void_p]
        lib.pn_reset.restype = None
        lib.pn_reset.argtypes = [ctypes.c_void_p]
        lib.pn_threads.restype = ctypes.c_int
        lib.pn_threads.argtypes = [ctypes.c_void_p]
        lib.pn_process.restype = ctypes.c_int
        lib.pn_process.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self.lib = lib
        self.path = os.fspath(path)


class NativeDoublePlateau:
    """영상 크기 하나에 대한 C++ 문맥. 프레임 순서대로 process() 를 부른다."""

    def __init__(
        self,
        library: PlateauNativeLibrary,
        width: int,
        height: int,
        threads: int,
        alpha: float,
    ) -> None:
        """C++ 문맥을 만든다.

        Args:
            library: 불러 온 라이브러리.
            width: 가로 픽셀 수.
            height: 세로 픽셀 수.
            threads: OpenMP 스레드 수. 0 이면 OpenMP 기본값.
            alpha: EMA 계수.

        Raises:
            RuntimeError: 문맥을 만들지 못했을 때.
        """
        self._lib = library.lib
        handle = self._lib.pn_create(width, height, threads, alpha)
        if not handle:
            raise RuntimeError("pn_create failed")
        self._handle: int | None = handle
        self.width = width
        self.height = height
        self.threads = int(self._lib.pn_threads(handle))
        self._factors = (ctypes.c_double * 2)()
        self._timings = (ctypes.c_double * 2)()

    @property
    def last_factors(self) -> tuple[float, float]:
        """마지막 프레임에 쓴 (상한, 하한) 계수."""
        return (self._factors[0], self._factors[1])

    @property
    def last_timings_ms(self) -> tuple[float, float]:
        """마지막 프레임의 (계수 추정, Plateau 변환) 시간 (ms)."""
        return (self._timings[0], self._timings[1])

    def reset(self) -> None:
        """EMA 상태를 지운다."""
        self._lib.pn_reset(self._require_handle())

    def process(self, raw: npt.NDArray[np.uint16]) -> npt.NDArray[np.uint8]:
        """프레임 한 장을 변환한다.

        Args:
            raw: (height, width) RAW14 영상. 연속이 아니면 복사한다.

        Returns:
            같은 크기의 8 비트 영상.

        Raises:
            ValueError: 크기가 문맥과 다를 때.
            RuntimeError: close() 뒤에 불렀을 때.
        """
        if raw.shape != (self.height, self.width):
            raise ValueError("frame size differs from the native context")
        contiguous = np.ascontiguousarray(raw, dtype=np.uint16)
        out = np.empty(contiguous.shape, np.uint8)
        self._lib.pn_process(
            self._require_handle(),
            contiguous.ctypes.data,
            out.ctypes.data,
            self._factors,
            self._timings,
        )
        return out

    def close(self) -> None:
        """C++ 문맥을 지운다. 두 번 불러도 된다."""
        if self._handle is not None:
            self._lib.pn_destroy(self._handle)
            self._handle = None

    def _require_handle(self) -> int:
        if self._handle is None:
            raise RuntimeError("native context is closed")
        return self._handle

    def __del__(self) -> None:
        """남은 C++ 문맥을 지운다."""
        self.close()
