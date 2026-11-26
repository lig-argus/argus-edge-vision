# 통제기 (Qt Quick). onboard와 별도인 독립 CMake 프로젝트이다.
# MAVLink 메시지는 common/mavlink/generated/의 C 헤더로 다룬다 (MAVSDK를 쓰지 않는다).
# 구조와 모듈 규칙: README.md "통제기 빌드", CONTRIBUTING.md 0.4, 모듈별 허용 의존은 cmake/module_rules.cmake
#
# 의존성: Qt 6.9 이상(Core, Gui, Network, Qml, Quick, Quick3D, Svg, ShaderTools, Test),
#        GStreamer 1.24 이상(gstreamer, app, video, rtp. pkg-config 로 찾는다)
# 빌드 (macOS):   cmake -S gcs -B build/gcs -DCMAKE_PREFIX_PATH=/opt/homebrew && cmake --build build/gcs
# 빌드 (Windows): cmake -S gcs -B build/gcs -DCMAKE_PREFIX_PATH=C:/Qt/6.9.3/msvc2022_64 && cmake --build build/gcs --config Release
# 테스트:         ctest --test-dir build/gcs
cmake_minimum_required(VERSION 3.16)
project(argus_gcs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # clang-tidy가 읽는 compile_commands.json
set(CMAKE_AUTOMOC ON)

# ShaderTools: Quick3D 의 셰이더를 빌드할 때 필요하다
find_package(Qt6 6.9 REQUIRED COMPONENTS Core Gui Network Qml Quick Quick3D Svg ShaderTools Test)

# RTSP 영상 수신 (docs/spec/video.md 1장). onboard 의 영상 송신부와 같은 GStreamer 1.24 이상
find_package(PkgConfig REQUIRED)
pkg_check_modules(ARGUS_GCS_GSTREAMER REQUIRED IMPORTED_TARGET
                  gstreamer-1.0>=1.24 gstreamer-app-1.0 gstreamer-video-1.0 gstreamer-rtp-1.0)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/mavlink ${CMAKE_CURRENT_BINARY_DIR}/mavlink)

# 경고 설정 (CONTRIBUTING.md 0.2 코딩 컨벤션). 라이브러리·실행 파일·테스트가 모두 이어받는다.
# -Wswitch-enum: switch 가 enum 값을 하나라도 빠뜨리면 경고한다 (default 가 있어도). enum 에 값을 추가하면
# 그 값을 처리해야 하는 switch 를 컴파일러가 모두 알려 준다. MSVC 는 C4061, C4062 가 같은 경고다
add_library(argus_gcs_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(argus_gcs_warnings INTERFACE -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum)
elseif(MSVC)
    target_compile_options(argus_gcs_warnings INTERFACE /W4 /utf-8 /w44061 /w44062)
endif()

include(CTest)
include(cmake/ArgusGcsModule.cmake)
include(cmake/module_rules.cmake)

# 모듈 (폴더 하나가 모듈 하나. 의존하는 모듈보다 뒤에 추가한다)
add_subdirectory(core)
add_subdirectory(core_testing)
add_subdirectory(mavlink_connection)
add_subdirectory(mavlink_connection_testing)
add_subdirectory(net)
add_subdirectory(link)
add_subdirectory(theme)
add_subdirectory(brand)
add_subdirectory(rtsp)
add_subdirectory(rtsp_gstreamer)
add_subdirectory(video)
add_subdirectory(video_rtsp)
add_subdirectory(app)

# 모듈의 include 가 선언한 의존을 넘지 않는지 확인한다
argus_gcs_write_module_list(${CMAKE_CURRENT_BINARY_DIR}/argus_gcs_modules.cmake)
add_test(NAME gcs_module_deps_test
         COMMAND ${CMAKE_COMMAND} -DMODULE_LIST=${CMAKE_CURRENT_BINARY_DIR}/argus_gcs_modules.cmake
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_module_deps.cmake)
