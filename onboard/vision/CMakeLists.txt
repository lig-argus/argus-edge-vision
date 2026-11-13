# 탐지(vision) 모듈의 C++ 부분. 탐지 작업자 자체는 Python(python/, 이 프로젝트에서 빌드하지 않음)이다.
# - argus_plateau_native: Double Plateau adaptive 전처리의 C++17 + OpenMP 라이브러리
#   (python/native/plateau_native.cpp). Python 이 ctypes 로 부른다 (python/argus_vision/plateau_native.py).
#   출력 파일은 build/onboard/vision/libplateau_native.so
# - OpenMP: Ubuntu 의 g++ 는 포함, macOS 는 brew install libomp
find_package(OpenMP REQUIRED)

add_library(argus_plateau_native SHARED python/native/plateau_native.cpp)
set_target_properties(argus_plateau_native PROPERTIES OUTPUT_NAME plateau_native)
# 곱셈과 덧셈을 합치지 않아(FMA 없음) double 계산이 NumPy 와 같게 반올림된다 (출력이 Python 경로와 같음)
target_compile_options(argus_plateau_native PRIVATE -ffp-contract=off)
target_link_libraries(argus_plateau_native PRIVATE OpenMP::OpenMP_CXX argus_onboard_warnings)
