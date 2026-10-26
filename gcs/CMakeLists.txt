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

# 통신 클래스들이 쓰는 인터페이스 (IMavlinkConnection, IClock)와 작은 구현 (docs/design/uml/class_gcs.md).
# IMavlinkConnection 은 QObject(signal)라 moc 가 필요하므로 STATIC 라이브러리다.
# 헤더는 include/argus/gcs/ 에 두고 #include "argus/gcs/<파일>.hpp" 로 부른다. namespace 는 argus::gcs
add_library(argus_gcs_interfaces STATIC
    include/argus/gcs/mavlink_connection.hpp
    include/argus/gcs/clock.hpp
    src/mavlink_connection.cpp
    src/steady_clock.cpp)
target_include_directories(argus_gcs_interfaces PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_gcs_interfaces PUBLIC Qt6::Core argus_mavlink argus_gcs_warnings)

add_executable(argus_gcs src/main.cpp)
target_link_libraries(argus_gcs PRIVATE Qt6::Widgets argus_gcs_interfaces argus_gcs_warnings)

# Qt Test: test/<이름>_test.cpp 마다 실행 파일 하나와 add_test 하나. 테스트용 구현(Fake*)은 test/ 에 둔다
include(CTest)
# Q_OBJECT 가 있는 테스트용 헤더(Fake*)는 SOURCES 에 함께 적어야 AUTOMOC 가 moc 를 만든다
function(argus_gcs_add_test name)
    cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})
    add_executable(${name} ${ARG_SOURCES})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(${name} PRIVATE argus_gcs_interfaces Qt6::Test argus_gcs_warnings)
    add_test(NAME ${name} COMMAND ${name})
endfunction()
argus_gcs_add_test(gcs_mavlink_connection_test
                   SOURCES test/mavlink_connection_test.cpp test/fake_mavlink_connection.hpp)
argus_gcs_add_test(gcs_clock_test SOURCES test/clock_test.cpp test/fake_clock.hpp)
