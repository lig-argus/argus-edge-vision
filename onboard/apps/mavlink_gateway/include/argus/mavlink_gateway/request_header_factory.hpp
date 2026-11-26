// mavlink_gateway 가 추종 제어에 보내는 요청(follow/command)의 공통 필드 생성
// 규약: docs/spec/process_interfaces.md 3장 (요청 공통 필드: command_id, session_id, seq, fc_epoch,
//      valid_until_monotonic_ns)
// 사용: GcsLinkMonitor (두절 정지), TrackStartHandler, TrackStopHandler, ScanStartHandler (다음
// 작업)
#ifndef ARGUS_ONBOARD_REQUEST_HEADER_FACTORY_HPP_
#define ARGUS_ONBOARD_REQUEST_HEADER_FACTORY_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>

#include "argus/core/clock.hpp"
#include "argus/mavlink_gateway/fc_epoch_source.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// command_id 와 seq 는 이 객체에서 1 부터 요청마다 1 씩 올린다. 요청자가 mavlink_gateway 하나이므로
// 한 프로세스 안에서 고유하며, 다시 시작한 경우는 session_id 로 구분한다
class RequestHeaderFactory {
public:
    // session_id 는 mavlink_gateway 가 시작할 때 만든 세션 (follow/status 의 onboard_session).
    // clock, fc_epoch 는 이 객체보다 오래 유지되어야 한다
    RequestHeaderFactory(std::string session_id, const core::IClock& clock,
                         const IFcEpochSource& fc_epoch)
        : session_id_(std::move(session_id)), clock_(clock), fc_epoch_(fc_epoch) {}

    // 새 요청의 공통 필드. valid_for 가 지나면 받는 쪽은 실행하지 않는다
    vehicle_api::RequestHeader next(std::chrono::milliseconds valid_for) {
        ++last_command_id_;
        ++last_seq_;
        vehicle_api::RequestHeader header;
        header.id.command_id = vehicle_api::CommandId{last_command_id_};
        header.id.session_id = session_id_;
        header.id.seq = last_seq_;
        header.id.fc_epoch = fc_epoch_.fc_epoch();
        header.valid_until_monotonic_ns =
            clock_.now_monotonic_ns() +
            std::chrono::duration_cast<std::chrono::nanoseconds>(valid_for).count();
        return header;
    }

    const std::string& session_id() const { return session_id_; }

private:
    std::string session_id_;
    const core::IClock& clock_;
    const IFcEpochSource& fc_epoch_;
    std::uint64_t last_command_id_ = 0;
    std::uint64_t last_seq_ = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_REQUEST_HEADER_FACTORY_HPP_
