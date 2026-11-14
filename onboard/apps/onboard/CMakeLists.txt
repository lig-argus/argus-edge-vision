# onboard 프로세스: FC 통신, 통제기 통신 (docs/design/uml/class_onboard.md). 통신 담당
# 실행 파일 이름: argus_onboard (main.cpp 로 add_executable). 그 밖의 소스는 라이브러리 argus_onboard_core(STATIC)에
# 추가하고 실행 파일과 테스트가 링크한다 (CONTRIBUTING.md 0.4). 모두 argus_onboard_warnings 를 링크한다.
# 헤더는 include/argus/onboard/, namespace 는 argus::onboard

# onboard 안 클래스들이 쓰는 인터페이스 헤더 (IMavlinkConnection, IMavlinkCommandHandler)
add_library(argus_onboard_interfaces INTERFACE)
add_library(argus::onboard_interfaces ALIAS argus_onboard_interfaces)
target_include_directories(argus_onboard_interfaces INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_onboard_interfaces INTERFACE argus::core argus_mavlink argus_onboard_warnings)

# FakeMavlinkConnection 은 onboard 테스트만 쓰므로 test/ 에 둔다
argus_add_test(onboard_fake_mavlink_connection_test SOURCES test/fake_mavlink_connection_test.cpp
               LIBS argus::onboard_interfaces argus::core_testing)
argus_add_test(onboard_command_responder_test SOURCES test/command_responder_test.cpp
               LIBS argus::onboard_interfaces)
