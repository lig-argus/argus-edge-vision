cmake_minimum_required(VERSION 3.16)
project(argus_onboard LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)
find_path(ZMQ_INCLUDE_DIR zmq.hpp REQUIRED)
find_library(ZMQ_LIBRARY zmq REQUIRED)
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp REQUIRED)
find_path(TOML_INCLUDE_DIR toml++/toml.h REQUIRED)
find_path(OPENCV_INCLUDE_DIR opencv2/core.hpp PATH_SUFFIXES opencv4 REQUIRED)
find_library(OPENCV_CORE opencv_core REQUIRED)
find_library(OPENCV_VIDEOIO opencv_videoio REQUIRED)
add_library(argus_core
    src/contracts/SystemClock.cpp
    src/adapters/OwnedProcess.cpp
    src/adapters/I3SdkFrameSource.cpp
    src/adapters/OpenCvFrameSource.cpp
    src/adapters/PythonVisionAdapter.cpp
    src/adapters/MessageBus.cpp
    src/application/EnvironmentProfile.cpp
    src/application/CompositionRoot.cpp
    src/application/Recorder.cpp
    src/application/PerceptionWorker.cpp)
target_include_directories(argus_core PUBLIC include ${ZMQ_INCLUDE_DIR} ${MSGPACK_INCLUDE_DIR} ${TOML_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR})
target_link_libraries(argus_core PUBLIC ${ZMQ_LIBRARY} ${OPENCV_CORE} ${OPENCV_VIDEOIO} Threads::Threads)
target_compile_options(argus_core PRIVATE -Wall -Wextra -Wpedantic)
add_executable(argus-onboard apps/OnboardMain.cpp)
target_link_libraries(argus-onboard PRIVATE argus_core)
set_target_properties(argus-onboard PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
add_executable(argus-frame-source apps/FrameSourceCapture.cpp src/contracts/SystemClock.cpp)
target_include_directories(argus-frame-source PRIVATE include ${OPENCV_INCLUDE_DIR})
target_link_libraries(argus-frame-source PRIVATE ${OPENCV_CORE} ${OPENCV_VIDEOIO})
set_target_properties(argus-frame-source PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
enable_testing()
add_executable(contract-tests tests/ContractsTest.cpp)
target_link_libraries(contract-tests PRIVATE argus_core)
add_test(NAME flow-contracts COMMAND contract-tests)
