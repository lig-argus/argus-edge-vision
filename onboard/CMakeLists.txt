# Raspberry Pi 에서 도는 프로세스(onboard, 영상 송신부, 추종 제어, 추적기)와 그 공용 라이브러리.
# 통제기(gcs/)와 별도인 CMake 프로젝트이다. 탐지 프로세스(vision/)는 Python 이라 여기서 빌드하지 않는다.
# 구조와 모듈 규칙: README.md "onboard 구조와 빌드", CONTRIBUTING.md 0.4
#
# 의존성 (Ubuntu 24.04 apt): libzmq3-dev cppzmq-dev libprotobuf-dev protobuf-compiler libgtest-dev
# 빌드:  cmake -S onboard -B build/onboard -G Ninja && cmake --build build/onboard
#        (macOS 는 README 의 protobuf 3.21.12 설치 뒤 -DCMAKE_PREFIX_PATH=$HOME/.local/protobuf-21)
# 테스트: ctest --test-dir build/onboard --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(argus_onboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # clang-tidy 가 읽는 compile_commands.json

# ZeroMQ C++ 바인딩(헤더만), protobuf(런타임과 protoc), gtest
# protobuf 는 설치 방식에 따라 설정 파일(protobuf-config.cmake, 소스 빌드)만 있거나 pkg-config(Ubuntu apt)만
# 있다. 설정 파일을 먼저 찾고 없으면 CMake 의 FindProtobuf 모듈로 찾는다
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)
find_package(cppzmq REQUIRED)
find_package(Protobuf REQUIRED)
find_package(GTest REQUIRED)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG OFF)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/mavlink ${CMAKE_CURRENT_BINARY_DIR}/mavlink)

# 경고 설정 (CONTRIBUTING.md 0.2 코딩 컨벤션). 모든 라이브러리·실행 파일·테스트가 이어받는다
add_library(argus_onboard_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(argus_onboard_warnings INTERFACE -Wall -Wextra -Wpedantic -Wshadow)
endif()

enable_testing()
include(cmake/ArgusTest.cmake)

# 공용 라이브러리 (프로세스들이 함께 쓴다)
add_subdirectory(libs/core)
add_subdirectory(libs/core_testing)
add_subdirectory(libs/bus)
add_subdirectory(libs/bus_testing)
add_subdirectory(libs/protocol)
add_subdirectory(libs/perception_api)
add_subdirectory(libs/vehicle_api)
add_subdirectory(libs/vehicle_api_testing)
add_subdirectory(libs/vehicle_api_zmq)

# 실행 파일 (프로세스 하나에 폴더 하나)
add_subdirectory(apps/onboard)
add_subdirectory(apps/video_sender)
add_subdirectory(apps/follow_controller)
add_subdirectory(apps/tracker)
