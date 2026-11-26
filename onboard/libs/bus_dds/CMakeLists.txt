# bus 인터페이스의 Fast DDS 구현: DdsBus (bus::IBus), DdsTypeTable (docs/design/uml/class_mavlink_gateway.md 1절)
# Fast DDS 로 통신하는 실행 파일만 링크한다. onboard/CMakeLists.txt 의 ARGUS_BUILD_BUS_DDS 가 ON 일 때만 빌드한다
# (기본 빌드는 Fast DDS 없이 된다). Fast DDS 와 Fast CDR 의 설치는 README.md
# 헤더는 include/argus/bus_dds/ 에 두고 #include "argus/bus_dds/<파일>.hpp" 로 부른다. namespace 는 argus::bus_dds
#
# 테스트용 IDL 타입(test/idl/test_text.idl)의 C++ 코드는 fastddsgen 4.3.0(Fast DDS 3.6.2 와 짝인 버전)으로
# 생성해 test/generated/ 에 커밋한다. CI 와 빌드에는 fastddsgen(Java)이 필요 없다. IDL 을 바꾸면
# onboard/libs/bus_dds/test/idl/ 에서 다시 생성한다:
#   fastddsgen -replace -ppDisable -d ../generated test_text.idl
find_package(fastcdr 2 REQUIRED)
find_package(fastdds 3 REQUIRED)

add_library(argus_bus_dds STATIC src/dds_bus.cpp src/fastdds_specific.cpp src/wakeup_pipe.cpp)
add_library(argus::bus_dds ALIAS argus_bus_dds)
target_include_directories(argus_bus_dds PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# fastdds 는 PUBLIC 이다. dds_type_table.hpp 에 TypeSupport 가 나오므로 사용하는 쪽도 Fast DDS 헤더가 필요하다
target_link_libraries(argus_bus_dds PUBLIC argus::bus argus::core fastdds fastcdr argus_onboard_warnings)

# fastddsgen 이 생성한 테스트용 타입. 생성 코드는 우리 경고 설정과 clang-tidy 검사에서 제외한다
add_library(argus_bus_dds_test_types STATIC
            test/generated/test_textPubSubTypes.cxx test/generated/test_textTypeObjectSupport.cxx)
target_include_directories(argus_bus_dds_test_types SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/test/generated)
target_link_libraries(argus_bus_dds_test_types PUBLIC fastdds fastcdr)

# 같은 프로세스 안 전달 (Fast DDS 의 기본 설정)
argus_add_test(bus_dds_test SOURCES test/dds_bus_test.cpp
               LIBS argus::bus_dds argus_bus_dds_test_types)
# 토픽별 발행자 QoS 와 위반 로그. DdsBus 를 사용하지 않는 상대를 Fast DDS 의 엔티티로 직접 만든다.
# 같은 프로세스 안 전달을 끄므로 따로 실행한다
argus_add_test(bus_dds_qos_test SOURCES test/dds_bus_qos_test.cpp
               LIBS argus::bus_dds argus_bus_dds_test_types)
# 전송 계층을 거치는 전달. 같은 프로세스 안 전달을 끄는 설정이 프로세스 전체에 적용되므로 따로 실행한다
argus_add_test(bus_dds_transport_test SOURCES test/dds_bus_transport_test.cpp
               LIBS argus::bus_dds argus_bus_dds_test_types)
