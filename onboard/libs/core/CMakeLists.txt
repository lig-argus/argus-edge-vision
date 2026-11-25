# 프로세스 공통 기반: IClock, SteadyClock, IPoller, IFdWatcher, ITimerScheduler, PollPoller, Subscription,
# HandlerList, EventLoop
# (docs/design/uml/class_mavlink_gateway.md 1절). 통신 종류(ZeroMQ, MAVLink)를 모른다.
# 헤더는 include/argus/core/ 에 두고 #include "argus/core/<파일>.hpp" 로 부른다. namespace 는 argus::core
add_library(argus_core STATIC src/event_loop.cpp src/poll_poller.cpp)
add_library(argus::core ALIAS argus_core)
target_include_directories(argus_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_core PUBLIC argus_onboard_warnings)

argus_add_test(core_steady_clock_test SOURCES test/steady_clock_test.cpp LIBS argus::core)
argus_add_test(core_subscription_test SOURCES test/subscription_test.cpp LIBS argus::core)
argus_add_test(core_handler_list_test SOURCES test/handler_list_test.cpp LIBS argus::core)
# EventLoop 테스트는 FakeClock, FakePoller 를 쓴다 (core_testing 이 core 를 함께 링크)
argus_add_test(core_event_loop_test SOURCES test/event_loop_test.cpp LIBS argus::core_testing)
# PollPoller 테스트는 pipe 로 fd 이벤트를 만들고, budget 과 타이머는 FakeClock 으로 확인한다
argus_add_test(core_poll_poller_test SOURCES test/poll_poller_test.cpp LIBS argus::core_testing)
