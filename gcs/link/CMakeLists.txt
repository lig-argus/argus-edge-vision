# 기체 상태 해석, 링크 판단, 명령 송신과 응답 처리 (VehicleLink 와 그 부품).
# Qt Core 와 MAVLink 헤더만 쓴다. 송수신은 IMavlinkConnection, 시각은 IClock 으로 받는다.
# VehicleLink 는 QObject(signal)라 moc 가 필요하므로 헤더를 소스에 함께 적는다.
# namespace 는 argus::gcs::link
argus_gcs_add_library(link
                      SOURCES include/argus/gcs/link/vehicle_link.hpp
                              src/command_encoder.cpp
                              src/command_policy.cpp
                              src/command_sender.cpp
                              src/command_traits.cpp
                              src/detection_store.cpp
                              src/flight_conditions.cpp
                              src/flight_env_params.cpp
                              src/link_monitor.cpp
                              src/manual_control.cpp
                              src/mode_change_watch.cpp
                              src/param_profile.cpp
                              src/px4_mode.cpp
                              src/status_text.cpp
                              src/telemetry_decoder.cpp
                              src/vehicle_link.cpp
                              src/video_stream_request.cpp
                      DEPS mavlink_connection core
                      QT Core
                      EXTERNAL mavlink)

argus_gcs_add_test(gcs_link_command_encoder_test SOURCES test/command_encoder_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_command_policy_test SOURCES test/command_policy_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_command_sender_test SOURCES test/command_sender_test.cpp LIBS argus::gcs::link argus::gcs::mavlink_connection_testing)
argus_gcs_add_test(gcs_link_command_traits_test SOURCES test/command_traits_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_detection_store_test SOURCES test/detection_store_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_flight_conditions_test SOURCES test/flight_conditions_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_flight_env_params_test SOURCES test/flight_env_params_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_link_monitor_test SOURCES test/link_monitor_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_manual_control_test SOURCES test/manual_control_test.cpp LIBS argus::gcs::link argus::gcs::mavlink_connection_testing)
argus_gcs_add_test(gcs_link_mode_change_watch_test SOURCES test/mode_change_watch_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_param_profile_test SOURCES test/param_profile_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_px4_mode_test SOURCES test/px4_mode_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_status_text_test SOURCES test/status_text_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_telemetry_decoder_test SOURCES test/telemetry_decoder_test.cpp LIBS argus::gcs::link)
argus_gcs_add_test(gcs_link_vehicle_link_test SOURCES test/vehicle_link_test.cpp LIBS argus::gcs::link argus::gcs::mavlink_connection_testing argus::gcs::core_testing)
argus_gcs_add_test(gcs_link_video_stream_request_test SOURCES test/video_stream_request_test.cpp LIBS argus::gcs::link)
