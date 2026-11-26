cmake_minimum_required(VERSION 3.16)
project(identity_quality_checks LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
find_package(Threads REQUIRED)
add_library(identity_core STATIC
 ${ARGUS_ROOT}/src/argus/control/target/target_manager.cpp
 ${ARGUS_ROOT}/src/argus/control/target/target_reacquirer.cpp
 ${ARGUS_ROOT}/src/argus/tracking/motion/short_motion_estimator.cpp)
target_include_directories(identity_core PUBLIC ${ARGUS_ROOT}/src ${ARGUS_ROOT}/vendor/vmy ${ARGUS_ROOT}/vendor/vmy/eigen ${ARGUS_ROOT}/vendor/yaw/msgpack/include)
target_link_libraries(identity_core PUBLIC Threads::Threads)
if(MSVC)
 target_compile_options(identity_core PUBLIC /utf-8)
 target_compile_options(identity_core PRIVATE /W4)
else()
 target_compile_options(identity_core PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
endif()
enable_testing()
foreach(name identity_quality selected_continuity identity_swap_continuity target_manager_import)
 if(name STREQUAL "identity_quality")
  add_executable(${name}_test ${name}_test.cpp)
 else()
  add_executable(${name}_test ${ARGUS_ROOT}/tests/yaw/contracts/${name}_test.cpp)
 endif()
 target_link_libraries(${name}_test PRIVATE identity_core)
 if(MSVC)
  target_compile_options(${name}_test PRIVATE /UNDEBUG)
 else()
  target_compile_options(${name}_test PRIVATE -UNDEBUG)
 endif()
 add_test(NAME ${name} COMMAND ${name}_test)
endforeach()

# Compile the production consumers of TargetMemory with the native Windows toolchain.
add_library(pc_follow_compile OBJECT
 ${ARGUS_ROOT}/src/argus/control/target/target_selector.cpp
 ${ARGUS_ROOT}/src/argus/control/guidance/flight_guidance_controller.cpp
 ${ARGUS_ROOT}/src/argus/control/runtime/flight_control_runtime.cpp
 ${ARGUS_ROOT}/src/argus/runtime/config/bus_following_config.cpp
 ${ARGUS_ROOT}/src/argus/runtime/follow/bus_following_application.cpp)
target_link_libraries(pc_follow_compile PRIVATE identity_core)
target_include_directories(pc_follow_compile PRIVATE ${ARGUS_ROOT}/vendor/yaw/zmq/include)
target_compile_definitions(pc_follow_compile PRIVATE MSGPACK_NO_BOOST ARGUS_FOLLOW_B=1)
