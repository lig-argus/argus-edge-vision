# 통제기와 mavlink_gateway 사이의 MAVLink UDP 연결 (UdpMavlinkConnection). IMavlinkConnection 과
# IMavlinkConnectionStatus 의 구현이다. 구체 구현을 만드는 app 만 이 모듈을 링크한다.
# namespace 는 argus::gcs::net
argus_gcs_add_library(net
                      SOURCES src/mavlink_stream_parser.cpp
                              src/udp_peer_socket.cpp
                              src/udp_mavlink_connection.cpp
                              src/endpoint_parser.cpp
                              include/argus/gcs/net/udp_peer_socket.hpp
                              include/argus/gcs/net/udp_mavlink_connection.hpp
                      DEPS mavlink_connection core
                      QT Core Network
                      EXTERNAL mavlink)

argus_gcs_add_test(gcs_net_mavlink_stream_parser_test SOURCES test/mavlink_stream_parser_test.cpp
                   LIBS argus::gcs::net argus::gcs::core_testing)
argus_gcs_add_test(gcs_net_udp_peer_socket_test SOURCES test/udp_peer_socket_test.cpp
                   LIBS argus::gcs::net argus::gcs::core_testing)
argus_gcs_add_test(gcs_net_udp_mavlink_connection_test SOURCES test/udp_mavlink_connection_test.cpp
                   LIBS argus::gcs::net argus::gcs::core_testing)
argus_gcs_add_test(gcs_net_endpoint_parser_test SOURCES test/endpoint_parser_test.cpp
                   LIBS argus::gcs::net)
