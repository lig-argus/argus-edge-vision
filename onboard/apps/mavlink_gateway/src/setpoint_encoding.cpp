#include "setpoint_encoding.hpp"

#include <cstdint>
#include <initializer_list>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway::detail {
namespace {

// POSITION_TARGET_TYPEMASK 의 비트들을 합친다. 비트 연산은 부호 없는 타입으로 한다
constexpr std::uint16_t mask_of(std::initializer_list<POSITION_TARGET_TYPEMASK> bits) {
    unsigned int mask = 0;
    for (const POSITION_TARGET_TYPEMASK bit : bits) {
        mask |= static_cast<unsigned int>(bit);
    }
    return static_cast<std::uint16_t>(mask);
}

// 속도와 yaw 회전 속도만 사용: 위치, 가속도, yaw 는 사용하지 않는 값으로 표시
constexpr std::uint16_t kVelocityAndYawRate =
    mask_of({POSITION_TARGET_TYPEMASK_X_IGNORE, POSITION_TARGET_TYPEMASK_Y_IGNORE,
             POSITION_TARGET_TYPEMASK_Z_IGNORE, POSITION_TARGET_TYPEMASK_AX_IGNORE,
             POSITION_TARGET_TYPEMASK_AY_IGNORE, POSITION_TARGET_TYPEMASK_AZ_IGNORE,
             POSITION_TARGET_TYPEMASK_YAW_IGNORE});

// 위치와 yaw 만 사용: 속도, 가속도, yaw 회전 속도는 사용하지 않는 값으로 표시
constexpr std::uint16_t kPositionAndYaw =
    mask_of({POSITION_TARGET_TYPEMASK_VX_IGNORE, POSITION_TARGET_TYPEMASK_VY_IGNORE,
             POSITION_TARGET_TYPEMASK_VZ_IGNORE, POSITION_TARGET_TYPEMASK_AX_IGNORE,
             POSITION_TARGET_TYPEMASK_AY_IGNORE, POSITION_TARGET_TYPEMASK_AZ_IGNORE,
             POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE});

// FC 는 time_boot_ms 를 사용하지 않는다
constexpr std::uint32_t kUnusedTimeBootMs = 0;

}  // namespace

mavlink_message_t encode_velocity_setpoint(const vehicle_api::VelocitySetpoint& setpoint) {
    mavlink_message_t message{};
    mavlink_msg_set_position_target_local_ned_pack_chan(
        kVehicleSysid, kOnboardCommandCompid, kCommandChannel, &message, kUnusedTimeBootMs,
        kVehicleSysid, kFcCompid, MAV_FRAME_BODY_NED, kVelocityAndYawRate, 0.0F, 0.0F, 0.0F,
        static_cast<float>(setpoint.forward_m_s), static_cast<float>(setpoint.right_m_s),
        static_cast<float>(setpoint.down_m_s), 0.0F, 0.0F, 0.0F, 0.0F,
        static_cast<float>(setpoint.yaw_rate_rad_s));
    return message;
}

mavlink_message_t encode_position_setpoint(double north_m, double east_m, double down_m,
                                           double yaw_rad) {
    mavlink_message_t message{};
    mavlink_msg_set_position_target_local_ned_pack_chan(
        kVehicleSysid, kOnboardCommandCompid, kCommandChannel, &message, kUnusedTimeBootMs,
        kVehicleSysid, kFcCompid, MAV_FRAME_LOCAL_NED, kPositionAndYaw, static_cast<float>(north_m),
        static_cast<float>(east_m), static_cast<float>(down_m), 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        static_cast<float>(yaw_rad), 0.0F);
    return message;
}

}  // namespace argus::mavlink_gateway::detail
