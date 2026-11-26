// 추종 제어에 보낸 follow/command 의 결과를 통제기 명령의 COMMAND_ACK 로 전달
// 규약: docs/spec/icd.md 4.1 (IN_PROGRESS 와 최종 결과), 4.2 (SCAN_START 의 progress),
//      docs/spec/process_interfaces.md 5.1.2 (command_results), 5.2, 5.2.1 (outcome 과 COMMAND_ACK)
// 사용: TrackStartHandler, ScanStartHandler, TrackStopHandler. main 이 하나를 생성하여 tick() 을
//      kFollowCommandTrackerTickPeriod 주기 작업으로 등록
#ifndef ARGUS_ONBOARD_FOLLOW_COMMAND_TRACKER_HPP_
#define ARGUS_ONBOARD_FOLLOW_COMMAND_TRACKER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink_gateway/follow_command_policy.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/vehicle_api/follow_control_client.hpp"
#include "argus/vehicle_api/follow_status_events.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// follow/command 의 응답 대기 시간. 요청의 유효 시간(valid_until)도 이 값으로 정한다
inline constexpr std::chrono::milliseconds kFollowCommandReplyTimeout{1000};

// mavlink_gateway 가 직접 정한 거부를 보낸다. STATUSTEXT 는 icd 4.1 에 따라 DENIED 에만 송신하고,
// TEMPORARILY_REJECTED 의 원인은 로그에만 남긴다
void send_rejection(CommandResponder& responder, const CommandRejection& rejection);

// tick() 의 호출 주기 (follow/status 두절 확인)
inline constexpr std::chrono::milliseconds kFollowCommandTrackerTickPeriod{1000};

// 명령 하나를 다음 순서로 처리한다.
// 1. IN_PROGRESS 송신. progress 는 SCAN_START 이면 0, 그 밖은 kProgressUnknown
// 2. follow/command 요청과 응답 대기 (kFollowCommandReplyTimeout)
//    - 응답 없음 → FAILED "추종 제어 응답 없음"
//    - Rejected → TEMPORARILY_REJECTED. 원인은 로그에만 남긴다 (icd 4.1 의 STATUSTEXT 대상이 아님).
//      mavlink_gateway 가 DENIED 조건을 모두 검사하므로 추종 제어의 거부는 상태 조건이다
//    - Succeeded → ACCEPTED, Failed → FAILED (원인을 STATUSTEXT)
//    - InProgress → 3 으로
// 3. follow/status 의 command_results 에서 같은 command_id 의 결과를 대기. 결과의 처리는 2 와 같다.
//    대기 시간의 상한은 두지 않는다 (스캔은 제한 시간 p3 까지 소요). follow/status 를
//    kFollowStatusTimeoutNs 동안 받지 못하면 FAILED "추종 제어 상태 두절".
//    SCAN_START 는 InProgress 인 동안 follow/status 의 scan 으로 progress 를 갱신한다.
// onboard_session 이 이 mavlink_gateway 의 세션이 아닌 follow/status 의 결과는 사용하지 않는다
// (mavlink_gateway 가 다시 시작하면 command_id 를 1 부터 다시 센다)
class FollowCommandTracker {
public:
    // 받은 handler 로 follow/command 를 요청하는 함수. IFollowControlClient 의 반환값을 돌려준다.
    // track() 안에서 한 번 부르고 보관하지 않으므로 호출자의 지역 변수를 참조로 담아도 된다
    using SendRequest =
        std::function<core::Subscription(vehicle_api::IFollowControlClient::ResultHandler)>;

    // onboard_session 은 RequestHeaderFactory 의 session_id.
    // 참조로 받은 객체들은 이 객체보다 오래 유지되어야 한다
    FollowCommandTracker(const core::IClock& clock,
                         const vehicle_api::IFollowStatusSource& status_source,
                         vehicle_api::IFollowStatusEvents& status_events,
                         std::string onboard_session);

    FollowCommandTracker(const FollowCommandTracker&) = delete;
    FollowCommandTracker& operator=(const FollowCommandTracker&) = delete;
    FollowCommandTracker(FollowCommandTracker&&) = delete;
    FollowCommandTracker& operator=(FollowCommandTracker&&) = delete;
    ~FollowCommandTracker() = default;

    // command_id 는 send 가 보낼 요청의 command_id. scan_yaw_range_rad 는 SCAN_START 의 yaw 범위
    // (progress 계산), 그 밖의 명령은 nullopt
    void track(vehicle_api::CommandId command_id, std::optional<double> scan_yaw_range_rad,
               CommandResponder responder, const SendRequest& send);

    // follow/status 두절을 확인하고 끝난 명령을 정리한다. kFollowCommandTrackerTickPeriod 마다
    // 호출한다
    void tick();

private:
    struct Entry {
        CommandResponder responder;
        core::Subscription reply;
        std::optional<double> scan_yaw_range_rad;
        std::uint8_t progress = kProgressUnknown;  // 마지막으로 보낸 IN_PROGRESS 의 progress

        // 추종 제어가 InProgress 로 응답한 시각. 응답 전이면 nullopt
        std::optional<std::int64_t> accepted_ns;
    };

    void on_reply(std::uint64_t command_id, const std::optional<vehicle_api::FollowResult>& reply);
    void on_status(const vehicle_api::ReceivedFollowStatus& received);
    // status 에 entry 의 결과가 있으면 응답을 보낸다
    void apply_status(std::uint64_t command_id, Entry& entry,
                      const vehicle_api::FollowStatus& status);
    // 최종 결과를 보낸 명령을 지운다. 추종 제어의 응답 handler 안에서는 부르지 않는다 (그 handler
    // 를 등록한 Subscription 이 entry 안에 있다)
    void erase_completed();

    const core::IClock& clock_;
    const vehicle_api::IFollowStatusSource& status_source_;
    std::string onboard_session_;
    std::map<std::uint64_t, Entry> entries_;  // command_id → 진행 중인 명령
    std::optional<std::int64_t> last_status_ns_;

    // 멤버 중 가장 먼저 소멸하도록 마지막에 선언한다
    core::Subscription status_subscription_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FOLLOW_COMMAND_TRACKER_HPP_
