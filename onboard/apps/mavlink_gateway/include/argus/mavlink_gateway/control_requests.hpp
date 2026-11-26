// 통제기의 제어권 요청과 해제를 처리하는 인터페이스
// 규약: docs/spec/icd.md 4.4 (MAV_CMD_ARGUS_REQUEST_OPERATOR_CONTROL), 5.1
// 구현: ControlAuthority
// 사용: RequestOperatorControlHandler
#ifndef ARGUS_ONBOARD_CONTROL_REQUESTS_HPP_
#define ARGUS_ONBOARD_CONTROL_REQUESTS_HPP_

#include <cstdint>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/gcs_link.hpp"

namespace argus::mavlink_gateway {

enum class ControlAction : std::uint8_t {
    Release,  // 해제 (p1 = 0)
    Request,  // 요청 (p1 = 1)
};

// 제어권을 조회만 하는 쪽은 IControlAuthority 를 사용한다
class IControlRequests {
public:
    IControlRequests() = default;
    virtual ~IControlRequests() = default;
    IControlRequests(const IControlRequests&) = delete;
    IControlRequests& operator=(const IControlRequests&) = delete;
    IControlRequests(IControlRequests&&) = delete;
    IControlRequests& operator=(IControlRequests&&) = delete;

    // sender 의 요청 또는 해제를 처리하고 COMMAND_ACK 의 결과를 반환한다.
    // 요청: 부여하면(이미 가진 경우 포함) ACCEPTED, 거부 조건이면 TEMPORARILY_REJECTED.
    // 해제: sender 가 제어권을 가졌으면 ACCEPTED, 아니면 DENIED
    virtual MAV_RESULT request(const GcsPeer& sender, ControlAction action) = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_CONTROL_REQUESTS_HPP_
