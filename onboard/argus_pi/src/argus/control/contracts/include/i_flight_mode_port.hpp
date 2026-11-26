// 운용자 명령으로 요청된 Offboard 전환만 FC 어댑터에 전달하는 경계다.
#pragma once
#include <optional>

namespace follow {

class IFlightModePort {
public:
    virtual ~IFlightModePort() = default;
    virtual bool start_offboard() = 0;
    virtual bool stop_offboard() = 0;
    // Synchronous ports return their ACK from the request method. Async ports
    // override these: nullopt=pending, false=failed ACK, true=accepted ACK.
    virtual std::optional<bool> start_result() const { return true; }
    virtual std::optional<bool> hold_result() const { return true; }
};

}  // namespace follow
