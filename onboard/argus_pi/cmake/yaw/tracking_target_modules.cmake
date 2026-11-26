# 기존 역할 소스를 선택/기억 전용 타깃으로 재사용한다. 기체 제어는 넣지 않는다.
set(ARGUS_TRACKING_SRC "${ARGUS_ROOT}")
function(argus_add_target_manager_core)
    add_library(target_manager_core STATIC
        ${ARGUS_TRACKING_SRC}/src/argus/control/target/target_manager.cpp
        ${ARGUS_TRACKING_SRC}/src/argus/control/target/target_reacquirer.cpp
        ${ARGUS_TRACKING_SRC}/src/argus/tracking/motion/short_motion_estimator.cpp
        ${ARGUS_TRACKING_SRC}/src/argus/control/target/target_selector.cpp)
    target_link_libraries(target_manager_core PUBLIC argus_headers Threads::Threads)
endfunction()
function(argus_add_target_monitor_bus)
    add_library(tracks_contract_bus STATIC
        ${ARGUS_TRACKING_SRC}/src/argus/tracking/contracts/tracks_v1_codec.cpp
        ${ARGUS_TRACKING_SRC}/src/argus/transport/zeromq/zeromq_tracks_bus.cpp)
    target_include_directories(tracks_contract_bus SYSTEM PRIVATE ${MSGPACK_INCLUDE_DIR} ${ZMQ_INCLUDE_DIR})
    # 2026-11-25 batch 2: ZeroMqTracksSubscriber 는 team_wire 로 받고 tracks_body 로 되살린다
    target_link_libraries(tracks_contract_bus PUBLIC argus_headers observation_bus argus_team_wire)
    add_library(target_monitor_bus STATIC
        ${ARGUS_TRACKING_SRC}/src/argus/transport/zeromq/target_tracks_worker.cpp
        ${ARGUS_TRACKING_SRC}/src/argus/commands/target/target_command_codec.cpp
        ${ARGUS_TRACKING_SRC}/src/argus/commands/target/zeromq_target_command_gateway.cpp)
    target_include_directories(target_monitor_bus SYSTEM PRIVATE ${MSGPACK_INCLUDE_DIR} ${ZMQ_INCLUDE_DIR})
    target_link_libraries(target_monitor_bus PUBLIC target_manager_core tracks_contract_bus)
endfunction()
function(argus_add_tracking_monitor)
    add_library(target_view_core STATIC
        ${ARGUS_TRACKING_SRC}/src/argus/diagnostics/tracking/target_view_snapshot.cpp
        ${ARGUS_TRACKING_SRC}/src/argus/diagnostics/tracking/target_view_codec.cpp)
    target_include_directories(target_view_core SYSTEM PRIVATE ${MSGPACK_INCLUDE_DIR})
    target_link_libraries(target_view_core PUBLIC target_manager_core)
    add_executable(tracking_target_monitor
        ${ARGUS_TRACKING_SRC}/apps/tracking_monitor/tracking_monitor_main.cpp
        ${ARGUS_TRACKING_SRC}/src/argus/runtime/config/tracking_monitor_config.cpp
        ${ARGUS_TRACKING_SRC}/src/argus/diagnostics/tracking/target_monitor_status.cpp)
    target_include_directories(tracking_target_monitor SYSTEM PRIVATE ${MSGPACK_INCLUDE_DIR} ${ZMQ_INCLUDE_DIR})
    target_link_libraries(tracking_target_monitor PRIVATE target_monitor_bus target_view_core)
endfunction()
