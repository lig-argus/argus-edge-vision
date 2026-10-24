# Raspberry Pi 에서 도는 프로세스(onboard, 영상 송신부, 추종 제어, 추적기)와 그 공용 라이브러리.
# 통제기(gcs/)와 별도인 CMake 프로젝트이다. 탐지 프로세스(vision/)는 Python 이라 여기서 빌드하지 않는다.
# 구조와 모듈 규칙: README.md "onboard 구조와 빌드", CONTRIBUTING.md 0.4
#
# 의존성 (Ubuntu 24.04 apt): libzmq3-dev cppzmq-dev libmsgpack-cxx-dev libgtest-dev
# 빌드:  cmake -S onboard -B build/onboard -G Ninja && cmake --build build/onboard
# 테스트: ctest --test-dir build/onboard --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(argus_onboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # clang-tidy 가 읽는 compile_commands.json

# msgpack-cxx 의 설정 파일이 Boost 를 찾는다. 옛 FindBoost 모듈 대신 Boost 가 설치한 BoostConfig.cmake 를 쓴다
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# ZeroMQ C++ 바인딩(헤더만), MessagePack C++(헤더만), gtest
find_package(cppzmq REQUIRED)
find_package(msgpack-cxx REQUIRED)
find_package(GTest REQUIRED)

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
add_subdirectory(libs/vehicle_api)
add_subdirectory(libs/vehicle_api_testing)
add_subdirectory(libs/vehicle_api_zmq)

# 실행 파일 (프로세스 하나에 폴더 하나)
add_subdirectory(apps/onboard)
add_subdirectory(apps/video_sender)
add_subdirectory(apps/follow_controller)
add_subdirectory(apps/tracker)
