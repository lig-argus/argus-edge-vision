#include "argus/gcs/link/flight_conditions.hpp"

#include <cmath>

#include "argus/mavlink.h"

namespace argus::gcs::link {

namespace {

bool linkUp(const ComponentLink& link) {
    return link.state == LinkState::Ok || link.state == LinkState::Delayed;
}

}  // namespace

FlightConditions::FlightConditions(const VehicleState& state, std::int64_t nowNs)
    : state_(state), nowNs_(nowNs) {}

bool FlightConditions::fcLinkUp() const { return linkUp(state_.fcLink); }

bool FlightConditions::gatewayLinkUp() const { return linkUp(state_.gatewayLink); }

bool FlightConditions::hasControl() const {
    return state_.gateway.received && state_.gateway.controlSysid == state_.gcsSysid;
}

bool FlightConditions::controlHeldByOther() const {
    return state_.gateway.received && state_.gateway.controlSysid != 0 &&
           state_.gateway.controlSysid != state_.gcsSysid;
}

bool FlightConditions::missionActive() const {
    if (!state_.gateway.received) {
        return false;
    }
    switch (state_.gateway.missionState) {
        case MissionState::Preparing:
        case MissionState::FollowActive:
        case MissionState::HoldTransition:
        case MissionState::ScanActive:
            return true;
        case MissionState::Idle:
        case MissionState::HoldWait:
        case MissionState::Manual:
        case MissionState::Fault:
            return false;
    }
    return false;
}

bool FlightConditions::missionReady() const {
    const MissionState mission = state_.gateway.missionState;
    return state_.gateway.received &&
           (mission == MissionState::Idle || mission == MissionState::HoldWait);
}

bool FlightConditions::missionStoppable() const {
    const MissionState mission = state_.gateway.missionState;
    return missionActive() || (state_.gateway.received &&
                               (mission == MissionState::Manual || mission == MissionState::Fault));
}

bool FlightConditions::gatewaySendingSetpoints() const {
    return missionActive() || state_.fc.mode == FlightMode::Offboard;
}

bool FlightConditions::pilotOverride() const {
    return state_.gateway.received && state_.gateway.missionState == MissionState::Manual;
}

bool FlightConditions::stationary() const {
    if (modeIs(FlightMode::Hold)) {
        return true;
    }
    return state_.fc.mode == FlightMode::Offboard && state_.gateway.received &&
           state_.gateway.missionState == MissionState::HoldWait;
}

bool FlightConditions::inAir() const { return state_.fc.landedState == MAV_LANDED_STATE_IN_AIR; }

// 이륙, 착륙 중도 지상이 아니다. landed_state 를 모르면 시동 상태로만 본다
bool FlightConditions::onGroundDisarmed() const {
    const std::uint8_t landed = state_.fc.landedState;
    const bool airborne = landed == MAV_LANDED_STATE_IN_AIR || landed == MAV_LANDED_STATE_TAKEOFF ||
                          landed == MAV_LANDED_STATE_LANDING;
    return !state_.fc.armed && !airborne;
}

// 받은 적이 없으면 altAmslM 이 NaN 이다 (테스트 시계는 0 부터라 시각으로는 구분하지 않는다)
bool FlightConditions::positionKnown() const {
    return std::isfinite(state_.fc.altAmslM) &&
           nowNs_ - state_.fc.globalPositionNs <= kPositionStaleNs;
}

// EKF 상태가 있으면 그것으로 본다: 수평 위치(상대 또는 절대)가 유효하고 실제 위치원을 쓰고 있을
// 때. 위치원 사용은 PRED_POS_HORIZ_REL 또는 절대 위치로 본다. CONST_POS_MODE 로는 판단하지 않는다:
// PX4 는 이 표시를 정지 상태에서도 켜므로 optical flow 를 쓰는 기체도 지상에서는 늘 켜져 있다.
// ESTIMATOR_STATUS 가 없으면 전역 위치로 본다
bool FlightConditions::horizontalPositionKnown() const {
    const FcState& fc = state_.fc;
    if (fc.estimatorSeen && nowNs_ - fc.estimatorNs <= kPositionStaleNs) {
        const bool horizontal = (fc.estimatorFlags & ESTIMATOR_POS_HORIZ_REL) != 0U ||
                                (fc.estimatorFlags & ESTIMATOR_POS_HORIZ_ABS) != 0U;
        const bool aided = (fc.estimatorFlags & ESTIMATOR_PRED_POS_HORIZ_REL) != 0U ||
                           (fc.estimatorFlags & ESTIMATOR_POS_HORIZ_ABS) != 0U;
        return horizontal && aided;
    }
    return positionKnown();
}

std::optional<float> FlightConditions::groundDistance() const {
    if (!std::isfinite(state_.fc.groundDistM) ||
        nowNs_ - state_.fc.groundDistNs > kGroundDistStaleNs) {
        return std::nullopt;
    }
    return state_.fc.groundDistM;
}

bool FlightConditions::manualMode() const {
    return modeIs(FlightMode::Manual) || modeIs(FlightMode::Altitude) ||
           modeIs(FlightMode::Position) || modeIs(FlightMode::Acro) ||
           modeIs(FlightMode::Stabilized);
}

bool FlightConditions::manualInputReady() const {
    const FcState& fc = state_.fc;
    return fc.rcPresent && fc.rcHealthy && nowNs_ - fc.rcStatusNs <= kRcStaleNs;
}

bool FlightConditions::rcReceiverActive() const {
    return state_.fc.rcChannels > 0 && nowNs_ - state_.fc.rcChannelsNs <= kRcStaleNs;
}

bool FlightConditions::sensorFollowable(std::uint8_t sensor) const {
    const auto found = state_.sensors.find(sensor);
    if (found == state_.sensors.end()) {
        return false;
    }
    const SensorStatus& status = found->second;
    return (status.flags & ARGUS_SENSOR_FLAGS_FOLLOWABLE) != 0 &&
           nowNs_ - status.receivedNs <= kSensorStaleNs;
}

}  // namespace argus::gcs::link
