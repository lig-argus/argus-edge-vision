// 별도 추적/대상 기억 실행의 설정. 비행 연결 주소나 제어값을 받지 않는다.
#pragma once
#include <string>
#include <vector>

#include "argus/control/target/include/target_selector.hpp"
namespace follow {
struct TrackingMonitorConfig {
    bool observations_input{
        false};  // Legacy observations option is rejected; only external tracks are supported.
    std::string observations_endpoint{"tcp://127.0.0.1:5556"};
    std::string tracks_endpoint{"tcp://127.0.0.1:5560"};
    std::string status_endpoint{"tcp://127.0.0.1:5561"};
    std::string command_endpoint{"tcp://127.0.0.1:5562"};
    std::vector<std::int32_t>
        class_ids;  // 빈 목록: 모델의 모든 class. 번호2를 IR에 강제하지 않는다.
    TargetSelectorConfig selector{};
    bool help{false};
};
TrackingMonitorConfig parse_tracking_monitor_options(int argc, char** argv);
}  // namespace follow
