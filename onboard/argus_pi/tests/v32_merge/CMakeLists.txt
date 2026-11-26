cmake_minimum_required(VERSION 3.16)
project(v32_main_merge_checks LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
find_package(Threads REQUIRED)
add_library(merge_core STATIC
 ${ARGUS_ROOT}/src/argus/control/target/target_manager.cpp
 ${ARGUS_ROOT}/src/argus/control/target/target_reacquirer.cpp
 ${ARGUS_ROOT}/src/argus/control/target/target_selector.cpp
 ${ARGUS_ROOT}/src/argus/tracking/motion/short_motion_estimator.cpp
 ${ARGUS_ROOT}/src/argus/runtime/config/bus_following_config.cpp
 ${ARGUS_ROOT}/src/argus/common/time/pose_history.cpp
 ${ARGUS_ROOT}/src/argus/control/geometry/calibrated_ground_geometry.cpp
 ${ARGUS_ROOT}/src/argus/control/geometry/camera_motion_guard.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/observed_rear_goal.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/pid_controller.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/yaw_rate_smoother.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/yaw_only_follower.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/predictive_follower.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/yaw_scan_controller.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/flight_guidance_controller.cpp
 ${ARGUS_ROOT}/src/argus/control/safety/flight_command_guard.cpp
 ${ARGUS_ROOT}/src/argus/control/safety/flight_control_watchdog.cpp
 ${ARGUS_ROOT}/src/argus/control/runtime/flight_control_runtime.cpp)
target_include_directories(merge_core PUBLIC ${ARGUS_ROOT}/src ${ARGUS_ROOT}/vendor/vmy ${ARGUS_ROOT}/vendor/vmy/eigen ${ARGUS_ROOT}/vendor/yaw/msgpack/include)
target_link_libraries(merge_core PUBLIC Threads::Threads)
if(MSVC)
 target_compile_options(merge_core PUBLIC /utf-8 /UNDEBUG)
else()
 target_compile_options(merge_core PUBLIC -UNDEBUG)
endif()
add_library(main_consumers OBJECT ${ARGUS_ROOT}/src/argus/runtime/follow/bus_following_application.cpp)
target_link_libraries(main_consumers PRIVATE merge_core)
target_include_directories(main_consumers PRIVATE ${ARGUS_ROOT}/vendor/yaw/zmq/include)
target_compile_definitions(main_consumers PRIVATE MSGPACK_NO_BOOST ARGUS_FOLLOW_B=1)
set(ARGUS_TEST_PROFILE_DIR "${ARGUS_ROOT}/config/follow" CACHE PATH "Materialized deployment profiles for config contracts")
enable_testing()
foreach(name v32_native_id_trust v32_sent_command_slew predictive_vertical_slew v3_follow_port)
 add_executable(${name}_test ${ARGUS_ROOT}/tests/follow_b/${name}_test.cpp)
 target_link_libraries(${name}_test PRIVATE merge_core)
endforeach()
add_test(NAME v32_native_id_priority COMMAND v32_native_id_trust_test ${ARGUS_TEST_PROFILE_DIR}/moving.config)
add_test(NAME v32_sent_command_slew COMMAND v32_sent_command_slew_test)
add_test(NAME retained_vertical_slew COMMAND predictive_vertical_slew_test)
add_test(NAME retained_profiles COMMAND v3_follow_port_test ${ARGUS_TEST_PROFILE_DIR})
add_executable(cv_ten_frame_test cv_ten_frame_test.cpp)
target_link_libraries(cv_ten_frame_test PRIVATE merge_core)
add_test(NAME v32_cv_history COMMAND cv_ten_frame_test)
foreach(name flight_control_runtime flight_command_guard selected_continuity identity_swap_continuity target_manager_import)
 add_executable(${name}_test ${ARGUS_ROOT}/tests/yaw/contracts/${name}_test.cpp)
 target_link_libraries(${name}_test PRIVATE merge_core)
 add_test(NAME ${name} COMMAND ${name}_test)
endforeach()
add_executable(identity_quality_test ${ARGUS_ROOT}/tests/identity_quality/identity_quality_test.cpp)
target_link_libraries(identity_quality_test PRIVATE merge_core)
add_test(NAME legacy_identity_quality COMMAND identity_quality_test)

add_executable(gap_integration_test gap_integration_test.cpp)
target_link_libraries(gap_integration_test PRIVATE merge_core)
add_test(NAME v32_gap_clock_and_history COMMAND gap_integration_test)

# 2026-11-09 짧은 입력 공백(NUC) 회귀: 실제 실내 moving 설정 사용, 실제 시간으로 약 50초
add_executable(short_gap_follow_test short_gap_follow_test.cpp)
target_link_libraries(short_gap_follow_test PRIVATE merge_core)
add_test(NAME short_gap_follow COMMAND short_gap_follow_test ${ARGUS_TEST_PROFILE_DIR}/moving.config)
