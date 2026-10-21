# gtest 테스트 등록. 모듈의 test/<이름>_test.cpp 하나에 한 번 호출한다.
#   argus_add_test(bus_publisher_test SOURCES test/publisher_test.cpp LIBS argus::bus)
# gtest_discover_tests 가 TEST() 하나하나를 ctest 항목으로 등록한다.
include(GoogleTest)

function(argus_add_test name)
    cmake_parse_arguments(ARG "" "" "SOURCES;LIBS" ${ARGN})
    add_executable(${name} ${ARG_SOURCES})
    target_link_libraries(${name} PRIVATE ${ARG_LIBS} GTest::gtest_main argus_onboard_warnings)
    gtest_discover_tests(${name})
endfunction()
