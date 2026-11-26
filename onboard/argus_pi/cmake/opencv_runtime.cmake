# Prefer installed headers/config; fallback matches Ubuntu 24.04 ABI 406 only.
find_package(OpenCV QUIET COMPONENTS core videoio)
if(NOT OpenCV_FOUND)
    find_path(ARGUS_OPENCV_HEADERS opencv2/core.hpp
        HINTS "${PROJECT_SOURCE_DIR}/vendor/pi_deps/include/opencv4" REQUIRED)
    find_library(ARGUS_OPENCV_CORE NAMES libopencv_core.so.406 REQUIRED)
    find_library(ARGUS_OPENCV_VIDEOIO NAMES libopencv_videoio.so.406 REQUIRED)
    set(OpenCV_INCLUDE_DIRS "${ARGUS_OPENCV_HEADERS}")
    set(OpenCV_LIBS "${ARGUS_OPENCV_CORE}" "${ARGUS_OPENCV_VIDEOIO}")
    message(STATUS "Using bundled OpenCV 4.6 headers and installed ABI 406 runtime")
endif()
