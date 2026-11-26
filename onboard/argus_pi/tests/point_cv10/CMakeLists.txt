cmake_minimum_required(VERSION 3.16)
project(v32_main_merge_checks LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(CANDIDATE "${ARGUS_ROOT}")
set(RATE_CANDIDATE "${ARGUS_ROOT}")
set(ARGUS_PROFILE_PATH "${ARGUS_ROOT}/build/v32-merge-check/materialized-profiles/moving.config" CACHE FILEPATH "Actual materialized profile")
find_package(Threads REQUIRED)
add_library(merge_core STATIC
 ${ARGUS_ROOT}/src/argus/control/target/target_manager.cpp
 ${ARGUS_ROOT}/src/argus/control/target/target_reacquirer.cpp
 ${CANDIDATE}/src/argus/control/target/target_selector.cpp
 ${ARGUS_ROOT}/src/argus/tracking/motion/short_motion_estimator.cpp
 ${ARGUS_ROOT}/src/argus/runtime/config/bus_following_config.cpp
 ${ARGUS_ROOT}/src/argus/common/time/pose_history.cpp
 ${ARGUS_ROOT}/src/argus/control/geometry/calibrated_ground_geometry.cpp
 ${ARGUS_ROOT}/src/argus/control/geometry/camera_motion_guard.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/observed_rear_goal.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/pid_controller.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/yaw_rate_smoother.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/yaw_only_follower.cpp
 ${RATE_CANDIDATE}/src/argus/control/guidance/predictive_follower.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/yaw_scan_controller.cpp
 ${CANDIDATE}/src/argus/control/guidance/flight_guidance_controller.cpp
 ${ARGUS_ROOT}/src/argus/control/safety/flight_command_guard.cpp
 ${ARGUS_ROOT}/src/argus/control/safety/flight_control_watchdog.cpp
 ${ARGUS_ROOT}/src/argus/control/runtime/flight_control_runtime.cpp)
target_include_directories(merge_core PUBLIC ${CANDIDATE}/src ${RATE_CANDIDATE}/src ${ARGUS_ROOT}/src ${ARGUS_ROOT}/vendor/vmy ${ARGUS_ROOT}/vendor/vmy/eigen ${ARGUS_ROOT}/vendor/yaw/msgpack/include)
target_link_libraries(merge_core PUBLIC Threads::Threads)
if(MSVC)
 target_compile_options(merge_core PUBLIC /utf-8 /UNDEBUG)
else()
 target_compile_options(merge_core PUBLIC -UNDEBUG)
endif()
add_library(main_consumers OBJECT ${CANDIDATE}/src/argus/runtime/follow/bus_following_application.cpp)
target_link_libraries(main_consumers PRIVATE merge_core)
target_include_directories(main_consumers PRIVATE ${ARGUS_ROOT}/vendor/yaw/zmq/include)
target_compile_definitions(main_consumers PRIVATE MSGPACK_NO_BOOST ARGUS_FOLLOW_B=1)
enable_testing()
add_executable(point_cv10_test point_cv10_test.cpp)
target_link_libraries(point_cv10_test PRIVATE merge_core)
add_test(NAME point_cv10_through_guard COMMAND point_cv10_test ${ARGUS_PROFILE_PATH} ${CMAKE_BINARY_DIR}/point-cv-wire.json)
