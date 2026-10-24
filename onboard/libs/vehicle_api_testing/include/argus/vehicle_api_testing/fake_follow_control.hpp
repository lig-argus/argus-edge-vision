// 테스트용 IFollowControl. 받은 명령을 기록하고, 테스트가 지정한 결과를 반환한다.
// onboard 의 ZeroMQ 어댑터(ZmqFollowControlServer) 테스트가 추종 제어 대신 쓴다
#ifndef ARGUS_VEHICLE_API_TESTING_FAKE_FOLLOW_CONTROL_HPP_
#define ARGUS_VEHICLE_API_TESTING_FAKE_FOLLOW_CONTROL_HPP_

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "argus/vehicle_api/follow_control.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api_testing {

enum class FollowCommandKind : std::uint8_t {
    TrackStart,
    Stop,
    ScanStart,
};

// FakeFollowControl 이 받은 명령 하나
struct FollowCommandRecord {
    FollowCommandKind kind = FollowCommandKind::Stop;
    vehicle_api::RequestHeader header;
    std::optional<vehicle_api::TrackTarget> target;  // TrackStart 일 때만
    std::optional<vehicle_api::StopReason> reason;   // Stop 일 때만
    std::optional<vehicle_api::ScanRequest> scan;    // ScanStart 일 때만
};

class FakeFollowControl : public vehicle_api::IFollowControl {
public:
    vehicle_api::FollowResult track_start(const vehicle_api::RequestHeader& header,
                                          const vehicle_api::TrackTarget& target) override {
        return record({FollowCommandKind::TrackStart, header, target, std::nullopt, std::nullopt});
    }
    vehicle_api::FollowResult stop(const vehicle_api::RequestHeader& header,
                                   vehicle_api::StopReason reason) override {
        return record({FollowCommandKind::Stop, header, std::nullopt, reason, std::nullopt});
    }
    vehicle_api::FollowResult scan_start(const vehicle_api::RequestHeader& header,
                                         const vehicle_api::ScanRequest& scan) override {
        return record({FollowCommandKind::ScanStart, header, std::nullopt, std::nullopt, scan});
    }

    // 다음 명령부터 반환할 결과(outcome, request_accepted, reason). id 는 요청의 RequestId 로
    // 채운다. 지정하지 않으면 수락하고 진행 중(InProgress)
    void set_result(vehicle_api::FollowResult result) { result_ = std::move(result); }

    const std::vector<FollowCommandRecord>& commands() const { return commands_; }

private:
    vehicle_api::FollowResult record(const FollowCommandRecord& command) {
        commands_.push_back(command);
        vehicle_api::FollowResult result = result_;
        result.id = command.header.id;
        return result;
    }

    static vehicle_api::FollowResult in_progress_result() {
        vehicle_api::FollowResult result;
        result.outcome = vehicle_api::FollowOutcome::InProgress;
        result.request_accepted = true;
        return result;
    }

    vehicle_api::FollowResult result_ = in_progress_result();
    std::vector<FollowCommandRecord> commands_;
};

}  // namespace argus::vehicle_api_testing

#endif  // ARGUS_VEHICLE_API_TESTING_FAKE_FOLLOW_CONTROL_HPP_
