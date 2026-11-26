// FC 표준 메시지(icd 2장)와 mavlink_gateway 상태 메시지(icd 3장)를 VehicleState 값으로 옮긴다.
// 값만 바꾸고 기록, 경고는 남기지 않는다. 바뀐 것을 알려야 하면 호출하는 쪽이 앞뒤 값을 비교한다.
// 시각은 호출하는 쪽이 monotonic ns 로 넘긴다 (이 모듈은 시계를 읽지 않는다)
#ifndef ARGUS_GCS_LINK_TELEMETRY_DECODER_HPP_
#define ARGUS_GCS_LINK_TELEMETRY_DECODER_HPP_

#include <cstdint>

#include "argus/gcs/link/link_protocol.hpp"
#include "argus/gcs/link/vehicle_state.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::link {

// GLOBAL_POSITION_INT, ESTIMATOR_STATUS 가 이보다 오래되면 해발 고도, 수평 위치를 모르는 것으로
// 본다 (TAKEOFF, Position 모드 차단). LOCAL_POSITION_NED 로 고도, 속도를 채우는 기준이기도 하다
inline constexpr std::int64_t kFcPositionStaleNs = 2 * kNsPerSecond;

// FC 상태 메시지 하나를 fc 에 반영한다. 상태 메시지가 아니면(STATUSTEXT, COMMAND_ACK 등) false
bool decodeFcState(FcState& fc, const mavlink_message_t& message, std::int64_t nowNs);

// mavlink_gateway 의 ARGUS_STATUS, ARGUS_SENSOR_STATUS 를 반영한다. 그 밖의 메시지면 false
bool decodeGatewayState(VehicleState& state, const mavlink_message_t& message, std::int64_t nowNs);

// ARGUS_STATUS.mission_state → MissionState. 범위 밖 값은 Fault 로 본다. 모르는 상태를 정상
// 상태(Idle 등)로 보이면 운용자가 이상을 놓치므로 경고로 표시되게 한다
MissionState toMissionState(std::uint8_t value);

// ARGUS_STATUS.target_state → TargetState. 범위 밖 값은 None 으로 본다. 표적이 있다고 잘못
// 보이면 운용자가 그 표적을 추종 중이라고 믿게 되므로 표적 없음으로 둔다
TargetState toTargetState(std::uint8_t value);

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_TELEMETRY_DECODER_HPP_
