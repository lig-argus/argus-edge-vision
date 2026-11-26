# MAVLink 연결 인터페이스 (IMavlinkConnection, IMavlinkConnectionStatus)와 값 타입(통계, 연결 대상),
# 메시지 이름 변환. 구현(UDP)은 net 모듈에 둔다.
# IMavlinkConnection 은 QObject(signal)라 moc 가 필요하므로 STATIC 라이브러리다.
# namespace 는 argus::gcs::mavlink_connection
argus_gcs_add_library(mavlink_connection
                      SOURCES include/argus/gcs/mavlink_connection/mavlink_connection.hpp
                              src/mavlink_connection.cpp
                              src/message_name.cpp
                      DEPS core
                      QT Core
                      EXTERNAL mavlink)

argus_gcs_add_test(gcs_mavlink_connection_test SOURCES test/mavlink_connection_test.cpp
                   LIBS argus::gcs::mavlink_connection_testing)
argus_gcs_add_test(gcs_mavlink_connection_message_name_test SOURCES test/message_name_test.cpp
                   LIBS argus::gcs::mavlink_connection)
