cmake_minimum_required(VERSION 3.16)
project(argus_follow_b LANGUAGES CXX)
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
# This configuration deliberately never discovers or links MAVSDK/MAVLink/UART.
include(${ARGUS_ROOT}/cmake/yaw/pi_tracking_base.cmake)
target_include_directories(argus_headers SYSTEM INTERFACE
    ${ARGUS_ROOT}/vendor/vmy ${ARGUS_ROOT}/vendor/vmy/eigen
    ${MSGPACK_INCLUDE_DIR} ${ZMQ_INCLUDE_DIR})
target_compile_definitions(argus_headers INTERFACE MSGPACK_NO_BOOST)
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
add_library(follow_b_transport STATIC
    ${ARGUS_ROOT}/src/argus/transport/follow_b/zeromq_follow_backend.cpp)
# 2026-11-25 batch 6a: FC 텔레메트리 6370 은 팀 [이름][protobuf] (team_wire 로 받는다)
target_link_libraries(follow_b_transport PUBLIC argus_headers observation_bus Threads::Threads
    PRIVATE argus_team_wire)
add_library(follow_b_application STATIC
    ${ARGUS_ROOT}/src/argus/runtime/config/bus_following_config.cpp
    ${ARGUS_ROOT}/src/argus/runtime/follow/bus_following_application.cpp
    ${ARGUS_ROOT}/src/argus/runtime/follow/follow_operator_gateway.cpp
    ${ARGUS_ROOT}/src/argus/diagnostics/status/system_status_v1_codec.cpp
    ${ARGUS_ROOT}/src/argus/diagnostics/status/zeromq_system_status_publisher.cpp)
target_link_libraries(follow_b_application PUBLIC following_core observation_bus target_monitor_bus)
# 2026-11-25 batch 8: 운용자 요청 6373 은 팀 [follow/command][protobuf] (team_wire 로 받고 답한다)
target_link_libraries(follow_b_application PRIVATE argus_team_wire)
add_executable(argus_follow_controller ${ARGUS_ROOT}/src/argus/runtime/follow/follow_main.cpp)
target_compile_definitions(argus_follow_controller PRIVATE ARGUS_FOLLOW_B=1)
target_link_libraries(argus_follow_controller PRIVATE follow_b_application follow_b_transport)

# Isolated ground diagnostic: no FC backend or actuator transport.
add_executable(argus_target_diagnostic
    ${ARGUS_ROOT}/src/argus/diagnostics/tracking/ground_diagnostic_main.cpp
    ${ARGUS_ROOT}/src/argus/runtime/config/bus_following_config.cpp)
target_link_libraries(argus_target_diagnostic PRIVATE following_core tracks_contract_bus)

add_library(follow_b_tracking STATIC
    ${ARGUS_ROOT}/vendor/vmy/lapjv/lapjv.cpp
    ${ARGUS_ROOT}/src/argus/tracking/ucmc/ucmc_tracker.cpp
    ${ARGUS_ROOT}/src/argus/tracking/ucmc/ucmc_kalman_track.cpp
    ${ARGUS_ROOT}/src/argus/tracking/ucmc/ucmc_tracker_adapter.cpp
    ${ARGUS_ROOT}/src/argus/perception/geometry/frame_transform.cpp
    ${ARGUS_ROOT}/src/argus/perception/geometry/pi_ground_projector.cpp
    ${ARGUS_ROOT}/src/argus/perception/geometry/static_camera_projector.cpp
    ${ARGUS_ROOT}/src/argus/common/time/frame_pose_alignment.cpp)
target_include_directories(follow_b_tracking SYSTEM PRIVATE ${ARGUS_ROOT}/vendor/vmy/lapjv)
target_link_libraries(follow_b_tracking PUBLIC argus_headers observation_bus)
add_executable(argus_follow_tracker ${ARGUS_ROOT}/src/argus/runtime/follow/pi_tracking_main.cpp)
target_compile_definitions(argus_follow_tracker PRIVATE ARGUS_FOLLOW_B_TRACKER=1)
target_link_libraries(argus_follow_tracker PRIVATE follow_b_tracking follow_b_transport argus_team_wire)
find_library(ARGUS_PREVIEW_ZLIB_LIBRARY NAMES z libz.so.1 REQUIRED)
add_executable(argus_preview_udp_relay
    ${ARGUS_ROOT}/src/argus/runtime/streaming/preview_udp_relay_main.cpp)
target_include_directories(argus_preview_udp_relay SYSTEM PRIVATE ${ARGUS_ROOT}/vendor/pi_deps/include)
# 2026-11-25 batch 4: 6374 의 vision/overlay/jpeg (protobuf) 를 받는다 (team_wire vision_body)
target_link_libraries(argus_preview_udp_relay PRIVATE observation_bus argus_team_wire
    ${ARGUS_PREVIEW_ZLIB_LIBRARY})
# 메인 GUI 영상 송신기 (C++, 영상+박스 v2). TurboJPEG가 있을 때만 만든다.
include(${ARGUS_ROOT}/cmake/video_avfr.cmake)
find_path(ARGUS_TURBOJPEG_INCLUDE_DIR turbojpeg.h)
find_library(ARGUS_TURBOJPEG_LIBRARY NAMES turbojpeg libturbojpeg.so.0)
if(ARGUS_TURBOJPEG_INCLUDE_DIR AND ARGUS_TURBOJPEG_LIBRARY)
    add_executable(argus_main_gcs_video
        ${ARGUS_ROOT}/src/argus/runtime/streaming/main_gcs_video_main.cpp)
    target_include_directories(argus_main_gcs_video SYSTEM PRIVATE
        ${ARGUS_ROOT}/vendor/pi_deps/include ${ARGUS_TURBOJPEG_INCLUDE_DIR})
    target_link_libraries(argus_main_gcs_video PRIVATE observation_bus argus_video
        argus_team_wire ${ARGUS_TURBOJPEG_LIBRARY})
else()
    message(STATUS "TurboJPEG not found: argus_main_gcs_video skipped")
endif()
if(BUILD_TESTING)
    # CMC clock/stream contracts are isolated: no serial, SDK camera, ARM or flight.
    add_executable(IndoorClockAlignmentTest ${ARGUS_ROOT}/tests/follow_b/indoor_clock_alignment_test.cpp)
    target_compile_options(IndoorClockAlignmentTest PRIVATE -UNDEBUG)
    target_link_libraries(IndoorClockAlignmentTest PRIVATE following_core follow_b_tracking)
    add_test(NAME IndoorClockAlignmentTest COMMAND IndoorClockAlignmentTest
        ${ARGUS_ROOT}/config/device/camera.json ${ARGUS_ROOT}/config/tracking/measurement.json)
    # 2026-11-09 열 외형 같은 대상 확인 (합성 RAW14·합성 트랙만, 카메라·FC 없음)
    add_executable(ThermalIdentityTest ${ARGUS_ROOT}/tests/follow_b/thermal_identity_test.cpp)
    target_compile_options(ThermalIdentityTest PRIVATE -UNDEBUG)
    target_link_libraries(ThermalIdentityTest PRIVATE follow_b_application)
    add_test(NAME ThermalIdentityTest COMMAND ThermalIdentityTest
        ${ARGUS_ROOT}/config/follow/yaw.config ${ARGUS_ROOT}/config/follow/moving.config)
    # 2026-11-12 실외 장착각 모드(30°/45°) 이름·투영 (합성 보정값만)
    add_executable(OutdoorMountProfileTest ${ARGUS_ROOT}/tests/follow_b/outdoor_mount_profile_test.cpp)
    target_compile_options(OutdoorMountProfileTest PRIVATE -UNDEBUG)
    target_link_libraries(OutdoorMountProfileTest PRIVATE follow_b_application)
    add_test(NAME OutdoorMountProfileTest COMMAND OutdoorMountProfileTest)
    # 2026-11-25 batch 4: argus-onboard 의 영상 6374 소유자 (VisionBusOwner) 시험 도구. 생산 코드와 같은
    # 소스를 직접 빌드한다 (perception 빌드는 BUILD_TESTING 을 끈다). selftest 는 장비와 worker 없이 가짜
    # ipc PUSH 와 inbox 로 넣고 raw SUB 와 고치지 않은 팀 ZmqBus 구독자로 받는다
    add_executable(vision_bus_probe ${ARGUS_ROOT}/tests/follow_b/vision_bus_probe.cpp
        ${ARGUS_ROOT}/src/argus/perception/detection/vision_bus_owner.cpp
        ${ARGUS_ROOT}/src/argus/transport/message_bus.cpp)
    target_compile_options(vision_bus_probe PRIVATE -UNDEBUG)
    target_link_libraries(vision_bus_probe PRIVATE argus_headers argus_team_wire Threads::Threads)
    add_test(NAME OnboardVisionBusTest COMMAND vision_bus_probe selftest)
    set_tests_properties(OnboardVisionBusTest PROPERTIES TIMEOUT 60)
    find_path(ARGUS_CPPZMQ_INCLUDE_DIR zmq.hpp HINTS ${ARGUS_ROOT}/vendor/pi_deps/include)
    if(ARGUS_CPPZMQ_INCLUDE_DIR)
        add_executable(ObservationClockTransportTest
            ${ARGUS_ROOT}/tests/follow_b/observation_clock_transport_test.cpp
            ${ARGUS_ROOT}/src/argus/transport/message_bus.cpp)
        target_include_directories(ObservationClockTransportTest SYSTEM PRIVATE ${ARGUS_CPPZMQ_INCLUDE_DIR})
        target_compile_options(ObservationClockTransportTest PRIVATE -UNDEBUG)
        target_link_libraries(ObservationClockTransportTest PRIVATE follow_b_transport observation_msgpack)
        add_test(NAME ObservationClockTransportTest COMMAND ObservationClockTransportTest)
    endif()
    add_executable(ProfileConfigCheck ${ARGUS_ROOT}/tests/follow_b/profile_config_check.cpp)
    target_compile_options(ProfileConfigCheck PRIVATE -UNDEBUG)
    target_link_libraries(ProfileConfigCheck PRIVATE follow_b_application)
    if(NOT Python3_FOUND)
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
    endif()
    add_test(NAME ProfileConfigCheck COMMAND ${Python3_EXECUTABLE}
        ${ARGUS_ROOT}/tests/follow_b/profile_config_effective_test.py
        ${ARGUS_ROOT} $<TARGET_FILE:ProfileConfigCheck>
        ${CMAKE_BINARY_DIR}/profile-config-effective.json)
    add_executable(PredictiveVerticalSlewTest ${ARGUS_ROOT}/tests/follow_b/predictive_vertical_slew_test.cpp)
    target_compile_options(PredictiveVerticalSlewTest PRIVATE -UNDEBUG)
    target_link_libraries(PredictiveVerticalSlewTest PRIVATE following_core)
    add_test(NAME PredictiveVerticalSlewTest COMMAND PredictiveVerticalSlewTest)
    # 2026-11-09 PAMPC point 고도 고정항·수직 무시 구간
    add_executable(PredictiveHeightAnchorTest
        ${ARGUS_ROOT}/tests/follow_b/predictive_height_anchor_test.cpp)
    target_compile_options(PredictiveHeightAnchorTest PRIVATE -UNDEBUG)
    target_link_libraries(PredictiveHeightAnchorTest PRIVATE following_core)
    add_test(NAME PredictiveHeightAnchorTest COMMAND PredictiveHeightAnchorTest)
    # 2026-11-10 가림 대응: 같은 ID 복귀 지면 확인·공백 비례 재획득 게이트
    add_executable(OcclusionGateTest ${ARGUS_ROOT}/tests/follow_b/occlusion_gate_test.cpp)
    target_compile_options(OcclusionGateTest PRIVATE -UNDEBUG)
    target_link_libraries(OcclusionGateTest PRIVATE following_core)
    add_test(NAME OcclusionGateTest COMMAND OcclusionGateTest)
    add_executable(GapCvFeedforwardTest ${ARGUS_ROOT}/tests/follow_b/gap_cv_feedforward_test.cpp)
    target_compile_options(GapCvFeedforwardTest PRIVATE -UNDEBUG)
    target_link_libraries(GapCvFeedforwardTest PRIVATE following_core)
    add_test(NAME GapCvFeedforwardTest COMMAND GapCvFeedforwardTest)
    # 2026-11-12 영상 프레임 단위 UDP 송신 (로컬 소켓만, 카메라·FC 없음)
    add_executable(UdpFrameSenderTest ${ARGUS_ROOT}/tests/follow_b/udp_frame_sender_test.cpp)
    target_include_directories(UdpFrameSenderTest PRIVATE ${ARGUS_ROOT}/src)
    add_test(NAME UdpFrameSenderTest COMMAND UdpFrameSenderTest)
    add_executable(LosRateFilterTest ${ARGUS_ROOT}/tests/follow_b/los_rate_filter_test.cpp)
    target_compile_options(LosRateFilterTest PRIVATE -UNDEBUG)
    target_link_libraries(LosRateFilterTest PRIVATE following_core)
    add_test(NAME LosRateFilterTest COMMAND LosRateFilterTest)
    add_executable(PredictiveRejectReasonTest
        ${ARGUS_ROOT}/tests/follow_b/predictive_reject_reason_test.cpp)
    target_compile_options(PredictiveRejectReasonTest PRIVATE -UNDEBUG)
    target_link_libraries(PredictiveRejectReasonTest PRIVATE following_core)
    add_test(NAME PredictiveRejectReasonTest COMMAND PredictiveRejectReasonTest)
    add_executable(CvReferenceRefreshTest ${ARGUS_ROOT}/tests/follow_b/cv_reference_refresh_test.cpp)
    target_compile_options(CvReferenceRefreshTest PRIVATE -UNDEBUG)
    target_link_libraries(CvReferenceRefreshTest PRIVATE following_core)
    add_test(NAME CvReferenceRefreshTest COMMAND CvReferenceRefreshTest)
    add_executable(PointTangentialTest ${ARGUS_ROOT}/tests/follow_b/point_tangential_test.cpp)
    target_compile_options(PointTangentialTest PRIVATE -UNDEBUG)
    target_link_libraries(PointTangentialTest PRIVATE following_core)
    add_test(NAME PointTangentialTest COMMAND PointTangentialTest)
    add_executable(ProjectionHysteresisTest ${ARGUS_ROOT}/tests/follow_b/projection_hysteresis_test.cpp)
    target_compile_options(ProjectionHysteresisTest PRIVATE -UNDEBUG)
    target_include_directories(ProjectionHysteresisTest PRIVATE ${ARGUS_ROOT}/src)
    add_test(NAME ProjectionHysteresisTest COMMAND ProjectionHysteresisTest)
    add_executable(ThermalAnchorGateTest ${ARGUS_ROOT}/tests/follow_b/thermal_anchor_gate_test.cpp)
    target_compile_options(ThermalAnchorGateTest PRIVATE -UNDEBUG)
    target_link_libraries(ThermalAnchorGateTest PRIVATE following_core)
    add_test(NAME ThermalAnchorGateTest COMMAND ThermalAnchorGateTest)
    # 2026-11-13 신원 판단 문맥(v36_identity_context) — 합성 트랙만
    add_executable(IdentityContextTest ${ARGUS_ROOT}/tests/follow_b/identity_context_test.cpp)
    target_compile_options(IdentityContextTest PRIVATE -UNDEBUG)
    target_link_libraries(IdentityContextTest PRIVATE following_core)
    add_test(NAME IdentityContextTest COMMAND IdentityContextTest)
    # 2026-11-16 같은 ID 판정 보류 중에도 GAP_CV 3초 유지 — 합성 트랙만
    add_executable(GapCvSameIdHoldTest ${ARGUS_ROOT}/tests/follow_b/gap_cv_same_id_hold_test.cpp)
    target_compile_options(GapCvSameIdHoldTest PRIVATE -UNDEBUG)
    target_link_libraries(GapCvSameIdHoldTest PRIVATE following_core)
    add_test(NAME GapCvSameIdHoldTest COMMAND GapCvSameIdHoldTest)
    add_executable(BearingRealignTest ${ARGUS_ROOT}/tests/follow_b/bearing_realign_test.cpp)
    target_compile_options(BearingRealignTest PRIVATE -UNDEBUG)
    target_link_libraries(BearingRealignTest PRIVATE following_core)
    add_test(NAME BearingRealignTest
        COMMAND BearingRealignTest ${ARGUS_ROOT}/config/follow/outdoor-predictive-moving.config)
    add_executable(PointRangeDeadbandTest ${ARGUS_ROOT}/tests/follow_b/point_range_deadband_test.cpp)
    target_compile_options(PointRangeDeadbandTest PRIVATE -UNDEBUG)
    target_link_libraries(PointRangeDeadbandTest PRIVATE following_core)
    add_test(NAME PointRangeDeadbandTest COMMAND PointRangeDeadbandTest)
    # 2026-11-17 실외 관측 만료 직전 명령 vz 0, 방위 재정렬 공백 초기화 회귀 시험.
    add_executable(OutdoorGapReserveTest ${ARGUS_ROOT}/tests/follow_b/outdoor_gap_reserve_test.cpp)
    target_compile_options(OutdoorGapReserveTest PRIVATE -UNDEBUG)
    target_link_libraries(OutdoorGapReserveTest PRIVATE follow_b_application following_core)
    add_test(NAME OutdoorGapReserveTest COMMAND OutdoorGapReserveTest
        ${ARGUS_ROOT}/config/follow/outdoor-predictive-moving.config
        ${ARGUS_ROOT}/config/follow/outdoor-low-predictive-moving.config
        ${ARGUS_ROOT}/config/follow/outdoor-m30-predictive-moving.config
        ${ARGUS_ROOT}/config/follow/outdoor-low-m30-predictive-moving.config)
    # 2026-11-18 실내 데모 모드: 설정 거부, yaw 전용 고도 유지, 송신 직전 vz 범위, vMy 짝.
    add_executable(DemoModeTest ${ARGUS_ROOT}/tests/follow_b/demo_mode_test.cpp)
    target_compile_options(DemoModeTest PRIVATE -UNDEBUG)
    target_link_libraries(DemoModeTest PRIVATE follow_b_application following_core vehicle_state_core)
    add_test(NAME DemoModeTest COMMAND DemoModeTest
        ${ARGUS_ROOT}/config/follow/demo-yaw.config
        ${ARGUS_ROOT}/config/follow/yaw.config
        ${ARGUS_ROOT}/config/follow/moving.config)
    add_executable(Demo2FixedYawTest ${ARGUS_ROOT}/tests/follow_b/demo2_fixed_yaw_test.cpp)
    target_compile_options(Demo2FixedYawTest PRIVATE -UNDEBUG)
    target_link_libraries(Demo2FixedYawTest PRIVATE follow_b_application following_core vehicle_state_core)
    add_test(NAME Demo2FixedYawTest COMMAND Demo2FixedYawTest
        ${ARGUS_ROOT}/config/follow/demo2-moving.config
        ${ARGUS_ROOT}/config/follow/demo2-yaw.config
        ${ARGUS_ROOT}/config/follow/moving.config
        ${ARGUS_ROOT}/config/follow/yaw.config)
    add_executable(OutdoorProfilesTest ${ARGUS_ROOT}/tests/follow_b/outdoor_profiles_test.cpp)
    target_compile_options(OutdoorProfilesTest PRIVATE -UNDEBUG)
    target_link_libraries(OutdoorProfilesTest PRIVATE follow_b_application follow_b_tracking)
    add_test(NAME OutdoorProfilesTest COMMAND OutdoorProfilesTest
        ${ARGUS_ROOT}/config/device/camera.json ${ARGUS_ROOT}/config/tracking/measurement-outdoor.json
        ${ARGUS_ROOT}/config/follow/outdoor-pid-moving.config)
    add_executable(CameraOnlyTrackingTest ${ARGUS_ROOT}/tests/follow_b/camera_only_tracking_test.cpp)
    target_compile_options(CameraOnlyTrackingTest PRIVATE -UNDEBUG)
    target_link_libraries(CameraOnlyTrackingTest PRIVATE following_core follow_b_tracking
        target_monitor_bus vehicle_state_core)
    add_test(NAME CameraOnlyTrackingTest COMMAND CameraOnlyTrackingTest
        ${ARGUS_ROOT}/tests/fixtures/tracking/static_camera.txt)
    add_executable(FloatIdentityTest ${ARGUS_ROOT}/tests/follow_b/float_identity_test.cpp)
    target_compile_options(FloatIdentityTest PRIVATE -UNDEBUG)
    target_link_libraries(FloatIdentityTest PRIVATE argus_headers Threads::Threads)
    add_test(NAME FloatIdentityTest COMMAND FloatIdentityTest)
    add_executable(FollowBTracksAdditiveTest
        ${ARGUS_ROOT}/tests/follow_b/tracks_additive_test.cpp)
    target_compile_options(FollowBTracksAdditiveTest PRIVATE -UNDEBUG)
    target_link_libraries(FollowBTracksAdditiveTest PRIVATE target_monitor_bus)
    add_test(NAME FollowBTracksAdditiveTest COMMAND FollowBTracksAdditiveTest)
    # 2026-11-19 PAMPC #5 tracks v5 선택 확장(producer 지면 기준) 계약 시험.
    add_executable(ProducerGroundWireTest ${ARGUS_ROOT}/tests/follow_b/producer_ground_wire_test.cpp)
    target_compile_options(ProducerGroundWireTest PRIVATE -UNDEBUG)
    target_link_libraries(ProducerGroundWireTest PRIVATE target_monitor_bus)
    add_test(NAME ProducerGroundWireTest COMMAND ProducerGroundWireTest)
    # 2026-11-19 PAMPC #3 rollout 기체 응답 이득, #5 지면 기준/투영 자세 (합성 입력만).
    add_executable(PampcModelFixTest ${ARGUS_ROOT}/tests/follow_b/pampc_model_fix_test.cpp)
    target_compile_options(PampcModelFixTest PRIVATE -UNDEBUG)
    target_link_libraries(PampcModelFixTest PRIVATE follow_b_application following_core)
    add_test(NAME PampcModelFixTest COMMAND PampcModelFixTest
        ${ARGUS_ROOT}/config/follow/moving.config
        ${ARGUS_ROOT}/config/follow/yaw.config)
    add_executable(FollowBTrackingStatusTest
        ${ARGUS_ROOT}/tests/follow_b/tracking_status_test.cpp)
    target_compile_options(FollowBTrackingStatusTest PRIVATE -UNDEBUG)
    target_link_libraries(FollowBTrackingStatusTest PRIVATE argus_headers)
    add_test(NAME FollowBTrackingStatusTest COMMAND FollowBTrackingStatusTest)
    add_executable(FollowPortsTest ${ARGUS_ROOT}/tests/follow_b/follow_ports_test.cpp)
    target_compile_options(FollowPortsTest PRIVATE -UNDEBUG)
    target_link_libraries(FollowPortsTest PRIVATE argus_headers)
    add_test(NAME FollowPortsTest COMMAND FollowPortsTest)
    # 2026-11-09 추적기 입력 최신 프레임만 꺼내기
    add_executable(LatestPixelReceiveTest ${ARGUS_ROOT}/tests/follow_b/latest_pixel_receive_test.cpp)
    target_compile_options(LatestPixelReceiveTest PRIVATE -UNDEBUG)
    target_link_libraries(LatestPixelReceiveTest PRIVATE follow_b_transport)
    add_test(NAME LatestPixelReceiveTest COMMAND LatestPixelReceiveTest)
    add_executable(FollowBCommandGatewayTest
        ${ARGUS_ROOT}/tests/follow_b/command_gateway_test.cpp)
    target_compile_options(FollowBCommandGatewayTest PRIVATE -UNDEBUG)
    target_link_libraries(FollowBCommandGatewayTest PRIVATE follow_b_application argus_team_wire)
    add_test(NAME FollowBCommandGatewayTest COMMAND FollowBCommandGatewayTest)
    set_tests_properties(FollowBCommandGatewayTest PROPERTIES TIMEOUT 20)
    add_executable(FollowBScanStateTest ${ARGUS_ROOT}/tests/follow_b/scan_state_test.cpp)
    target_compile_options(FollowBScanStateTest PRIVATE -UNDEBUG)
    target_link_libraries(FollowBScanStateTest PRIVATE following_core)
    add_test(NAME FollowBScanStateTest COMMAND FollowBScanStateTest)
    add_executable(FollowBTransportTest ${ARGUS_ROOT}/tests/yaw/contracts/zeromq_follow_backend_test.cpp)
    target_compile_options(FollowBTransportTest PRIVATE -UNDEBUG)
    target_link_libraries(FollowBTransportTest PRIVATE follow_b_transport argus_team_wire)
    # 2026-11-25 batch 6b: follow/status 시험 본문은 실기 기록 fixture (tests/protobuf/fixtures)
    target_compile_definitions(FollowBTransportTest PRIVATE
        ARGUS_PB_FIXTURES="${ARGUS_ROOT}/tests/protobuf/fixtures")
    add_test(NAME FollowBTransportTest COMMAND FollowBTransportTest)
    set_tests_properties(FollowBTransportTest PROPERTIES TIMEOUT 20)
    foreach(source IN ITEMS FlightGuidanceControllerTest FlightCommandGuardTest
            CalibratedGroundGeometryTest FlightControlRuntimeTest)
        string(REGEX REPLACE "([A-Z]+)([A-Z][a-z])" "\\1_\\2" source_file "${source}")
        string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" source_file "${source_file}")
        string(TOLOWER "${source_file}" source_file)
        add_executable(${source} ${ARGUS_ROOT}/tests/yaw/contracts/${source_file}.cpp)
        target_compile_options(${source} PRIVATE -UNDEBUG)
        target_link_libraries(${source} PRIVATE following_core vehicle_state_core)
        add_test(NAME ${source} COMMAND ${source})
    endforeach()
    if(Python3_FOUND)
        # 2026-11-25 batch 2: 시험 harness 는 tracking/tracks, tracking/status 를 protobuf 로 읽는다
        # (src/argus/transport/pb_json.py, tests/protobuf/team_tracks_wire.py)
        execute_process(COMMAND ${Python3_EXECUTABLE} -c "import zmq, msgpack, google.protobuf"
            RESULT_VARIABLE FOLLOW_B_PYTHON_READY OUTPUT_QUIET ERROR_QUIET)
        if(FOLLOW_B_PYTHON_READY EQUAL 0)
            add_test(NAME CameraOnlyProcessTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/camera_only_process_test.py
                ${ARGUS_ROOT} ${CMAKE_BINARY_DIR})
            set_tests_properties(CameraOnlyProcessTest PROPERTIES TIMEOUT 60)
            add_test(NAME ObservationSelectionProcessTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/observation_selection_process_test.py
                ${ARGUS_ROOT} ${CMAKE_BINARY_DIR})
            set_tests_properties(ObservationSelectionProcessTest PROPERTIES TIMEOUT 30)
            add_test(NAME FloatIdentityIntegrationTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/float_identity_integration_test.py
                ${ARGUS_ROOT} ${CMAKE_BINARY_DIR})
            set_tests_properties(FloatIdentityIntegrationTest PROPERTIES TIMEOUT 30)
            add_test(NAME FollowBProcessIntegrationTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/process_integration_test.py
                --build ${CMAKE_BINARY_DIR}
                --report ${CMAKE_BINARY_DIR}/follow-b-process-integration.json)
            set_tests_properties(FollowBProcessIntegrationTest PROPERTIES TIMEOUT 60)
            add_test(NAME FollowBIcdIntegrationTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/process_integration_test.py
                --build ${CMAKE_BINARY_DIR} --icd
                --report ${CMAKE_BINARY_DIR}/follow-b-icd-integration.json)
            set_tests_properties(FollowBIcdIntegrationTest PROPERTIES TIMEOUT 60)
            # 2026-11-17 누른 순간 프레임에 없는 대상의 추종 시작 대기, 탐지 복귀 시 시작, 10초 한도.
            add_test(NAME DeferredTrackStartProcessTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/process_integration_test.py
                --build ${CMAKE_BINARY_DIR} --icd --deferred-start
                --report ${CMAKE_BINARY_DIR}/deferred-track-start.json)
            set_tests_properties(DeferredTrackStartProcessTest PROPERTIES TIMEOUT 90)
            add_test(NAME FollowBShutdownDiagnosticTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/process_integration_test.py
                --build ${CMAKE_BINARY_DIR} --shutdown-during-follow
                --report ${CMAKE_BINARY_DIR}/follow-b-shutdown-diagnostic.json)
            set_tests_properties(FollowBShutdownDiagnosticTest PROPERTIES TIMEOUT 60)
            add_test(NAME PredictedReportingProcessTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/process_integration_test.py
                --build ${CMAKE_BINARY_DIR} --icd --process-v2 --prediction-gap
                --report ${CMAKE_BINARY_DIR}/predicted-reporting.json)
            set_tests_properties(PredictedReportingProcessTest PROPERTIES TIMEOUT 60)
            add_test(NAME ExplicitStopRecoveryProcessTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/process_integration_test.py
                --build ${CMAKE_BINARY_DIR} --icd --process-v2 --recovery
                --report ${CMAKE_BINARY_DIR}/explicit-stop-recovery.json)
            set_tests_properties(ExplicitStopRecoveryProcessTest PROPERTIES TIMEOUT 60)
            add_test(NAME FollowBProcessProtocolTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/process_integration_test.py
                --build ${CMAKE_BINARY_DIR} --icd --process-v2
                --report ${CMAKE_BINARY_DIR}/follow-b-process-protocol.json)
            set_tests_properties(FollowBProcessProtocolTest PROPERTIES TIMEOUT 60)
            add_test(NAME FollowBTrackingStatusIntegrationTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/tracking_status_integration_test.py
                ${ARGUS_ROOT} ${CMAKE_BINARY_DIR})
            set_tests_properties(FollowBTrackingStatusIntegrationTest PROPERTIES TIMEOUT 15)
            # 2026-11-25 batch 2: 추적기 기본 출력 6375 의 팀 프레임 형식과 팀 ZmqBus 구독자 해석.
            # 고정 포트 6375 를 쓰므로 따로 돌리고, 다른 프로세스가 쓰고 있으면 SKIP (77)
            add_executable(TeamTrackingBusProbe ${ARGUS_ROOT}/tests/follow_b/team_tracking_bus_probe.cpp)
            target_link_libraries(TeamTrackingBusProbe PRIVATE argus_team_wire)
            add_test(NAME TeamTrackingBusTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/follow_b/team_tracking_bus_test.py
                ${ARGUS_ROOT} ${CMAKE_BINARY_DIR})
            set_tests_properties(TeamTrackingBusTest PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE
                SKIP_RETURN_CODE 77)
            # 2026-11-25 batch 2 리뷰: real_fc B owner 는 tracks protobuf 해석기를 시작할 때 불러온다
            # (FC owner 스레드의 첫 tracks 프레임에서 불러오지 않는다). inproc 와 빈 포트만 쓴다
            add_test(NAME RealFcPbJsonPreloadTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/real_fc/test_pb_json_preload.py)
            set_tests_properties(RealFcPbJsonPreloadTest PROPERTIES TIMEOUT 30)
            # 2026-11-25 batch 4: 팀 vision 6374 의 소비자 시험 (임의 포트, 장비 없음).
            # 영상 준비 확인 (Python ensure, zmq 소켓만 가짜): 6374 소유, 예전 생산자 거부, 본문 거부
            add_test(NAME PreviewSupervisorVisionTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/integration/preview/test_preview_supervisor.py)
            set_tests_properties(PreviewSupervisorVisionTest PROPERTIES TIMEOUT 60)
            # I3U1 중계: protobuf overlay 를 받아 예전과 같은 JSON 바이트로 보내고 거부 본문은 보내지 않는다
            add_test(NAME PreviewUdpRelayVisionTest COMMAND ${Python3_EXECUTABLE}
                ${ARGUS_ROOT}/tests/integration/preview/test_virtual_udp_relay.py
                --relay $<TARGET_FILE:argus_preview_udp_relay>
                --report ${CMAKE_BINARY_DIR}/preview-udp-relay-vision.json)
            set_tests_properties(PreviewUdpRelayVisionTest PROPERTIES TIMEOUT 60)
            execute_process(COMMAND ${Python3_EXECUTABLE} -c "import cv2, numpy"
                RESULT_VARIABLE FOLLOW_B_PYTHON_CV2 OUTPUT_QUIET ERROR_QUIET)
            if(TARGET argus_main_gcs_video AND FOLLOW_B_PYTHON_CV2 EQUAL 0)
                # 메인 GUI 영상 송신기: VisionBusOwner (vision_bus_probe serve) -> AVFR, 거부 본문
                add_test(NAME MainGcsVideoVisionTest COMMAND ${Python3_EXECUTABLE}
                    ${ARGUS_ROOT}/tests/follow_b/main_gcs_video_vision_test.py
                    ${ARGUS_ROOT} ${CMAKE_BINARY_DIR})
                set_tests_properties(MainGcsVideoVisionTest PROPERTIES TIMEOUT 60)
            else()
                message(STATUS "MainGcsVideoVisionTest unavailable: argus_main_gcs_video or Python cv2")
            endif()
        else()
            message(STATUS "B process tests unavailable: Python zmq/msgpack/protobuf not installed")
        endif()
    endif()
endif()
option(ARGUS_WITH_DDS "Build DDS (Fast DDS 2.11) observation source for the tracker" OFF)
if(ARGUS_WITH_DDS)
    find_package(fastcdr REQUIRED)
    find_package(fastrtps REQUIRED)
    add_library(argus_dds_source STATIC
        ${ARGUS_ROOT}/src/argus/transport/dds/dds_observation_source.cpp
        ${ARGUS_ROOT}/src/argus/transport/dds/generated/ArgusObservation.cxx
        ${ARGUS_ROOT}/src/argus/transport/dds/generated/ArgusObservationPubSubTypes.cxx)
    target_include_directories(argus_dds_source PUBLIC ${ARGUS_ROOT}/src/argus/transport/dds/generated)
    target_compile_definitions(argus_dds_source PUBLIC ARGUS_WITH_DDS=1)
    # observation_source.hpp (ZeroMQ 경로) 가 team_wire vision_body 를 쓴다 (2026-11-25 batch 4)
    target_link_libraries(argus_dds_source PUBLIC argus_headers argus_team_wire fastrtps fastcdr)
    target_link_libraries(argus_follow_tracker PRIVATE argus_dds_source)
    add_executable(argus_dds_probe ${ARGUS_ROOT}/src/argus/transport/dds/dds_probe_main.cpp)
    target_link_libraries(argus_dds_probe PRIVATE argus_dds_source)
endif()

# 2026-11-16 시선각 필터 재생 검증 도구 (장비 미사용, 서비스 미등록)
add_executable(argus_los_rate_replay ${ARGUS_ROOT}/tests/follow_b/los_rate_replay.cpp)
target_link_libraries(argus_los_rate_replay PRIVATE follow_b_application)
