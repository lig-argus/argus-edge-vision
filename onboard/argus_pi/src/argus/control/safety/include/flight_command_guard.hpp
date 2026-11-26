// 최종 PX4 전송 직전에 만료·선택 epoch·실제 FC 모드·유한값을 검사한다.
#pragma once

#include <array>
#include <limits>
#include <mutex>

#include "argus/control/guidance/include/flight_guidance_controller.hpp"
#include "argus/vehicle/contracts/include/i_vehicle_port.hpp"
#include "argus/vehicle/contracts/include/i_vehicle_state_source.hpp"
#include "argus/vehicle/contracts/include/vehicle_flight_status.hpp"

namespace follow {

class FlightCommandGuard final {
public:
    enum class Result { Sent, SelectionChanged, Rejected };

    FlightCommandGuard(TargetSelector&, const IVehicleFlightStatusSource&, IVehiclePort&,
                       bool yaw_only = false, std::recursive_mutex* context_mutex = nullptr);
    Result send(const FlightGuidanceResult&, const TargetSelectionSnapshot&,
                Clock::time_point now = Clock::now());

    void set_yaw_only(bool value) { yaw_only_ = value; }
    void set_limits(std::array<double, 3> value) { limits_ = value; }
    // 2026-11-18 데모 모드: yaw 전용에서 범위 안쪽으로 향하는 제한된 vz만 허용한다.
    // pose_source가 없으면 vz != 0은 계속 거부한다.
    void set_yaw_only_altitude_band(YawOnlyAltitudeBand band,
                                    const IVehicleStateSource* pose_source) {
        altitude_band_ = band;
        pose_source_ = pose_source;
    }

private:
    TargetSelector& selector_;
    const IVehicleFlightStatusSource& flight_state_source_;
    IVehiclePort& vehicle_port_;
    bool yaw_only_;
    std::recursive_mutex* context_mutex_;
    YawOnlyAltitudeBand altitude_band_{};
    const IVehicleStateSource* pose_source_{nullptr};
    bool yaw_only_vertical_allowed(double vz_m_s, Clock::time_point now) const;
    std::array<double, 3> limits_{std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::infinity()};
};

}  // namespace follow
