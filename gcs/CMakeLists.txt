# 통제기 (Qt Widgets). onboard와 별도인 독립 CMake 프로젝트이다.
# MAVLink 메시지는 common/mavlink/generated/의 C 헤더로 다룬다 (MAVSDK를 쓰지 않는다).
#
# 빌드 (macOS):   cmake -S gcs -B build/gcs -DCMAKE_PREFIX_PATH=/opt/homebrew && cmake --build build/gcs
# 빌드 (Windows): cmake -S gcs -B build/gcs -DCMAKE_PREFIX_PATH=C:/Qt/6.x/msvc2022_64 && cmake --build build/gcs --config Release
# 시험:           ctest --test-dir build/gcs
cmake_minimum_required(VERSION 3.16)
project(argus_gcs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # clang-tidy가 읽는 compile_commands.json
set(CMAKE_AUTOMOC ON)

find_package(Qt6 6.2 REQUIRED COMPONENTS Core Gui Widgets Network Test)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/mavlink ${CMAKE_CURRENT_BINARY_DIR}/mavlink)

# 경고 설정 (CONTRIBUTING.md 0.2 코딩 컨벤션). 라이브러리·실행 파일·시험이 모두 이어받는다
add_library(argus_gcs_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(argus_gcs_warnings INTERFACE -Wall -Wextra -Wpedantic -Wshadow)
elseif(MSVC)
    target_compile_options(argus_gcs_warnings INTERFACE /W4 /utf-8)
endif()

add_executable(argus_gcs src/main.cpp)
target_link_libraries(argus_gcs PRIVATE Qt6::Widgets argus_mavlink argus_gcs_warnings)

# Qt Test: test/<이름>_test.cpp 마다 add_test. 첫 통신 클래스와 함께 추가한다
include(CTest)
