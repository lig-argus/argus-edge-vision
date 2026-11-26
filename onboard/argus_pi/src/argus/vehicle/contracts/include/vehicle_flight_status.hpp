// 제어 주기에서 확인하는 FC 연결·비행 모드 상태와 읽기 전용 공급 경계다.
#pragma once

namespace follow {

struct VehicleFlightStatus {
    bool connected{false};
    bool armed{false};
    bool in_air{false};
    bool offboard{false};
    bool hold{false};
    bool rc_override{false};
    bool position{false};
    bool ground_known{false};
    bool landed{false};
    bool offboard_hover{false};  // Owner-confirmed local hover; not PX4 AUTO_LOITER.
};

class IVehicleFlightStatusSource {
public:
    virtual ~IVehicleFlightStatusSource() = default;
    virtual VehicleFlightStatus read() const = 0;
};

}  // namespace follow
