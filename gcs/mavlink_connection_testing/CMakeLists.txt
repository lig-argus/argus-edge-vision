# mavlink_connection 의 테스트용 구현 (FakeMavlinkConnection). 통신 모듈들의 테스트가 링크한다.
# FakeMavlinkConnection 은 QObject 라 moc 가 필요하므로 헤더를 소스로 둔 STATIC 라이브러리다.
# 제품 실행 파일은 링크하지 않는다. namespace 는 argus::gcs::mavlink_connection_testing
argus_gcs_add_library(mavlink_connection_testing
                      SOURCES
                          include/argus/gcs/mavlink_connection_testing/fake_mavlink_connection.hpp
                      DEPS mavlink_connection core
                      QT Core
                      EXTERNAL mavlink)
