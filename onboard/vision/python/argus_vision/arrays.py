# 구현 모듈이 같이 쓰는 numpy 배열 형식.
# 저장소 CI 의 mypy 환경에는 numpy 가 없어 Any 로 본다 (pyproject.toml 의 mypy 예외).

import numpy as np
import numpy.typing as npt

type Raw14Image = npt.NDArray[np.uint16]
type Gray8Image = npt.NDArray[np.uint8]
