# 통신 인터페이스: IBus 가 만드는 ITopicPublisher, ITopicSubscriber, IRequester, IReplier
# (docs/design/uml/class_onboard.md 1절). 통신 구현(ZeroMQ, Fast DDS 등)을 교체해도 이 인터페이스는
# 바뀌지 않는다. bus 는 바이트만 나르고, 직렬화와 구조체 변환은 어댑터가 한다.
# 지금은 헤더만 있는 INTERFACE 라이브러리이다. ZeroMQ 구현(ZmqBus, #59)은 src/ 에 넣고 STATIC 으로 바꾸며,
# 그때 cppzmq 를 링크한다 (인터페이스만 사용하는 코드는 ZeroMQ 없이 빌드된다).
# 헤더는 include/argus/bus/ 에 두고 #include "argus/bus/<파일>.hpp" 로 부른다. namespace 는 argus::bus
add_library(argus_bus INTERFACE)
add_library(argus::bus ALIAS argus_bus)
target_include_directories(argus_bus INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_bus INTERFACE argus::core argus_onboard_warnings)
