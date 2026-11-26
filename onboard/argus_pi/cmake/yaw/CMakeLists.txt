cmake_minimum_required(VERSION 3.16)
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
project(argus_yaw LANGUAGES CXX)
# 순수 monitor의 공통 라이브러리와 계약을 그대로 사용한다. yaw 구성은 별도 묶음이다.
include(${CMAKE_CURRENT_LIST_DIR}/pi_tracking_base.cmake)
find_package(MAVSDK REQUIRED)
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES MAVSDK::mavsdk)
set(CMAKE_REQUIRED_DEFINITIONS -DMAVSDK_ENABLE_MAVLINK_C_API)
unset(ARGUS_HAS_SDK_EGRESS_HOOK CACHE)
check_cxx_source_compiles("\n#if __has_include(<mavsdk/mavsdk.hpp>)\n#include <mavsdk/mavsdk.hpp>\n#else\n#include <mavsdk/mavsdk.h>\n#endif\nint main(){ auto outgoing=&mavsdk::Mavsdk::intercept_outgoing_messages_async; auto incoming=&mavsdk::Mavsdk::intercept_incoming_messages_async; return outgoing && incoming ? 0 : 1; }" ARGUS_HAS_SDK_EGRESS_HOOK)
unset(CMAKE_REQUIRED_LIBRARIES)
unset(CMAKE_REQUIRED_DEFINITIONS)
if(NOT ARGUS_HAS_SDK_EGRESS_HOOK)
    message(FATAL_ERROR "Installed MAVSDK lacks the required incoming identity/outgoing lease hooks. Keep the guards; do not install or replace dependencies automatically.")
endif()
add_library(mavsdk_compat INTERFACE)
target_compile_definitions(mavsdk_compat INTERFACE MAVSDK_ENABLE_MAVLINK_C_API)
target_link_libraries(mavsdk_compat INTERFACE MAVSDK::mavsdk)
add_library(following_core STATIC
    ${ARGUS_ROOT}/src/argus/control/geometry/calibrated_ground_geometry.cpp
    ${ARGUS_ROOT}/src/argus/control/guidance/observed_rear_goal.cpp
    ${ARGUS_ROOT}/src/argus/control/guidance/pid_controller.cpp
    ${ARGUS_ROOT}/src/argus/control/guidance/yaw_rate_smoother.cpp
    ${ARGUS_ROOT}/src/argus/control/guidance/yaw_only_follower.cpp
    ${ARGUS_ROOT}/src/argus/control/guidance/predictive_follower.cpp
    ${ARGUS_ROOT}/src/argus/control/guidance/yaw_scan_controller.cpp
    ${ARGUS_ROOT}/src/argus/control/geometry/camera_motion_guard.cpp
    ${ARGUS_ROOT}/src/argus/control/guidance/flight_guidance_controller.cpp
    ${ARGUS_ROOT}/src/argus/control/safety/flight_command_guard.cpp
    ${ARGUS_ROOT}/src/argus/control/safety/flight_control_watchdog.cpp
    ${ARGUS_ROOT}/src/argus/control/runtime/flight_control_runtime.cpp)
target_link_libraries(following_core PUBLIC argus_headers target_manager_core Threads::Threads)
add_library(vehicle_state_core STATIC ${ARGUS_ROOT}/src/argus/common/time/pose_history.cpp)
target_link_libraries(vehicle_state_core PUBLIC argus_headers)
add_library(mavsdk_ports STATIC
    ${ARGUS_ROOT}/src/argus/vehicle/mavsdk/mavsdk_vehicle_port.cpp
    ${ARGUS_ROOT}/src/argus/vehicle/mavsdk/mavsdk_pose_source.cpp
    ${ARGUS_ROOT}/src/argus/vehicle/mavsdk/mavsdk_flight_status_source.cpp
    ${ARGUS_ROOT}/src/argus/vehicle/mavsdk/mavsdk_identity_guard.cpp
    ${ARGUS_ROOT}/src/argus/vehicle/mavsdk/mavsdk_offboard_mode_port.cpp)
target_link_libraries(mavsdk_ports PUBLIC argus_headers vehicle_state_core mavsdk_compat)
add_library(following_bus STATIC
    ${ARGUS_ROOT}/src/argus/diagnostics/status/system_status_v1_codec.cpp
    ${ARGUS_ROOT}/src/argus/diagnostics/status/zeromq_system_status_publisher.cpp)
target_include_directories(following_bus SYSTEM PRIVATE ${MSGPACK_INCLUDE_DIR} ${ZMQ_INCLUDE_DIR})
target_link_libraries(following_bus PUBLIC following_core target_monitor_bus)
add_library(bus_following_config STATIC ${ARGUS_ROOT}/src/argus/runtime/config/bus_following_config.cpp)
target_link_libraries(bus_following_config PUBLIC following_core)
if(BUILD_TESTING)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        add_executable(FileCommandOwnerTest ${ARGUS_ROOT}/tests/yaw/contracts/file_command_owner_test.cpp)
        target_compile_options(FileCommandOwnerTest PRIVATE -UNDEBUG)
        target_link_libraries(FileCommandOwnerTest PRIVATE argus_headers)
        add_test(NAME FileCommandOwnerTest COMMAND FileCommandOwnerTest)
        set_tests_properties(FileCommandOwnerTest PROPERTIES TIMEOUT 5)
    endif()
    add_executable(MavsdkLeaseTest ${ARGUS_ROOT}/tests/yaw/integration/mavsdk_lease_test.cpp)
    target_compile_options(MavsdkLeaseTest PRIVATE -UNDEBUG)
    target_link_libraries(MavsdkLeaseTest PRIVATE mavsdk_ports)
    add_test(NAME MavsdkLeaseTest COMMAND MavsdkLeaseTest)
    set_tests_properties(MavsdkLeaseTest PROPERTIES TIMEOUT 25)
    add_executable(YawProtocolTest ${ARGUS_ROOT}/tests/yaw/contracts/yaw_protocol_test.cpp)
    target_compile_options(YawProtocolTest PRIVATE -UNDEBUG)
    target_link_libraries(YawProtocolTest PRIVATE following_bus)
    add_test(NAME YawProtocolTest COMMAND YawProtocolTest)
    foreach(source IN ITEMS GroundYawBenchGuardTest FlightGuidanceControllerTest FlightCommandGuardTest CalibratedGroundGeometryTest FlightControlRuntimeTest BusFollowingConfigTest )
        string(REGEX REPLACE "([A-Z]+)([A-Z][a-z])" "\\1_\\2" source_file "${source}")
        string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" source_file "${source_file}")
        string(TOLOWER "${source_file}" source_file)
        add_executable(${source} ${ARGUS_ROOT}/tests/yaw/contracts/${source_file}.cpp)
        target_compile_options(${source} PRIVATE -UNDEBUG)
        target_link_libraries(${source} PRIVATE following_bus vehicle_state_core bus_following_config)
        add_test(NAME ${source} COMMAND ${source})
    endforeach()
endif()

# vMy is separate from the legacy yaw-only executable; both share guarded ports.
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    if(NOT EXISTS "${ARGUS_ROOT}/vendor/vmy/eigen/Eigen/Core")
        message(FATAL_ERROR "Eigen3 headers required; keep installed SDK/dependencies unchanged")
    endif()
    add_library(vmy_eigen INTERFACE)
    target_include_directories(vmy_eigen SYSTEM INTERFACE ${ARGUS_ROOT}/vendor/vmy/eigen)
    add_library(Eigen3::Eigen ALIAS vmy_eigen)
endif()
target_include_directories(argus_headers SYSTEM INTERFACE ${ARGUS_ROOT}/vendor/vmy)
target_include_directories(argus_headers SYSTEM INTERFACE ${ZMQ_INCLUDE_DIR} ${MSGPACK_INCLUDE_DIR})
target_compile_definitions(argus_headers INTERFACE MSGPACK_NO_BOOST)
add_library(vmy_lapjv STATIC ${ARGUS_ROOT}/vendor/vmy/lapjv/lapjv.cpp)
target_include_directories(vmy_lapjv SYSTEM PUBLIC ${ARGUS_ROOT}/vendor/vmy/lapjv)
add_library(vmy_tracking_core STATIC
    ${ARGUS_ROOT}/src/argus/tracking/ucmc/ucmc_tracker.cpp
    ${ARGUS_ROOT}/src/argus/tracking/ucmc/ucmc_kalman_track.cpp
    ${ARGUS_ROOT}/src/argus/tracking/ucmc/ucmc_tracker_adapter.cpp
    ${ARGUS_ROOT}/src/argus/perception/geometry/frame_transform.cpp
    ${ARGUS_ROOT}/src/argus/perception/geometry/pi_ground_projector.cpp
    ${ARGUS_ROOT}/src/argus/perception/geometry/static_camera_projector.cpp
    ${ARGUS_ROOT}/src/argus/common/time/frame_pose_alignment.cpp)
target_link_libraries(vmy_tracking_core PUBLIC argus_headers Eigen3::Eigen vmy_lapjv
    observation_bus mavsdk_compat)
add_executable(argus_vmy
    ${ARGUS_ROOT}/src/argus/runtime/follow/follow_main.cpp
    ${ARGUS_ROOT}/src/argus/runtime/follow/bus_following_application.cpp
    ${ARGUS_ROOT}/src/argus/runtime/follow/mavsdk_legacy_backend.cpp
    ${ARGUS_ROOT}/src/argus/runtime/follow/follow_operator_gateway.cpp)
target_compile_definitions(argus_vmy PRIVATE ARGUS_EXTERNAL_TRACKS_ONLY=1 ARGUS_VMY_NATIVE=1)
target_link_libraries(argus_vmy PRIVATE mavsdk_ports bus_following_config following_bus argus_team_wire)
add_executable(argus_vmy_command ${ARGUS_ROOT}/src/argus/runtime/follow/vmy_command_main.cpp)
target_include_directories(argus_vmy_command SYSTEM PRIVATE ${ZMQ_INCLUDE_DIR})
target_link_libraries(argus_vmy_command PRIVATE observation_bus)

# Transport only: no camera, inference, tracking or FC library is linked here.
find_library(VMY_ZLIB_LIBRARY NAMES z libz.so.1 REQUIRED)
add_executable(argus_preview_udp_relay ${ARGUS_ROOT}/src/argus/runtime/streaming/preview_udp_relay_main.cpp)
target_include_directories(argus_preview_udp_relay SYSTEM PRIVATE ${ARGUS_ROOT}/vendor/pi_deps/include)
target_link_libraries(argus_preview_udp_relay PRIVATE observation_bus argus_team_wire ${VMY_ZLIB_LIBRARY})
if(BUILD_TESTING)
    add_executable(VMyScanTest ${ARGUS_ROOT}/tests/yaw/contracts/vmy_scan_test.cpp)
    add_executable(FloatIdentityTest ${ARGUS_ROOT}/tests/follow_b/float_identity_test.cpp)
    target_compile_options(FloatIdentityTest PRIVATE -UNDEBUG)
    target_link_libraries(FloatIdentityTest PRIVATE argus_headers Threads::Threads)
    add_test(NAME FloatIdentityTest COMMAND FloatIdentityTest)
    target_compile_options(VMyScanTest PRIVATE -UNDEBUG)
    target_link_libraries(VMyScanTest PRIVATE following_core)
    add_test(NAME VMyScanTest COMMAND VMyScanTest)
endif()

add_executable(argus_vmy_pi_tracker ${ARGUS_ROOT}/src/argus/runtime/follow/pi_tracking_main.cpp)
target_link_libraries(argus_vmy_pi_tracker PRIVATE vmy_tracking_core argus_team_wire)

if(BUILD_TESTING)
    add_executable(VMyNativeTest
        ${ARGUS_ROOT}/tests/yaw/contracts/vmy_native_test.cpp
        ${ARGUS_ROOT}/src/argus/runtime/follow/bus_following_application.cpp
        ${ARGUS_ROOT}/src/argus/runtime/follow/mavsdk_legacy_backend.cpp
        ${ARGUS_ROOT}/src/argus/runtime/follow/follow_operator_gateway.cpp)
    target_compile_definitions(VMyNativeTest PRIVATE ARGUS_EXTERNAL_TRACKS_ONLY=1 ARGUS_VMY_NATIVE=1)
    target_compile_options(VMyNativeTest PRIVATE -UNDEBUG)
    target_link_libraries(VMyNativeTest PRIVATE vmy_tracking_core mavsdk_ports bus_following_config following_bus
        argus_team_wire)
    add_test(NAME VMyNativeTest COMMAND VMyNativeTest ${ARGUS_ROOT})
endif()
