#include "argus/mavlink_gateway/fc_rc_status_receiver.hpp"

#include <cstdint>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/mavlink_ids.hpp"

namespace argus::mavlink_gateway {
namespace {

// SYS_STATUS 의 present 와 health 비트가 모두 켜져 있으면 true
bool present_and_healthy(const mavlink_sys_status_t& status, std::uint32_t bit) {
    return (status.onboard_control_sensors_present & bit) != 0 &&
           (status.onboard_control_sensors_health & bit) != 0;
}

}  // namespace

FcRcStatusReceiver::FcRcStatusReceiver(IMavlinkConnection& connection, FcLinkTracker& link)
    : link_(link) {
    subscriptions_.push_back(
        connection.subscribe(MAVLINK_MSG_ID_RC_CHANNELS, [this](const mavlink_message_t& message) {
            if (is_from_fc(message)) {
                on_rc_channels(message);
            }
        }));
    subscriptions_.push_back(
        connection.subscribe(MAVLINK_MSG_ID_SYS_STATUS, [this](const mavlink_message_t& message) {
            if (is_from_fc(message)) {
                on_sys_status(message);
            }
        }));
    // 이전 fc_epoch 에서 받은 값을 새 연결의 값으로 쓰지 않도록 삭제한다
    subscriptions_.push_back(link_.subscribe_epoch_changes([this](std::uint64_t) {
        rc_channels_.reset();
        sensor_health_.reset();
    }));
}

void FcRcStatusReceiver::on_rc_channels(const mavlink_message_t& message) {
    mavlink_rc_channels_t fc{};
    mavlink_msg_rc_channels_decode(&message, &fc);
    RcChannels channels;
    channels.received_monotonic_ns = link_.on_fc_message();
    channels.fc_epoch = link_.fc_epoch();
    channels.channel_count = fc.chancount;
    channels.raw_us = {fc.chan1_raw,  fc.chan2_raw,  fc.chan3_raw,  fc.chan4_raw,  fc.chan5_raw,
                       fc.chan6_raw,  fc.chan7_raw,  fc.chan8_raw,  fc.chan9_raw,  fc.chan10_raw,
                       fc.chan11_raw, fc.chan12_raw, fc.chan13_raw, fc.chan14_raw, fc.chan15_raw,
                       fc.chan16_raw, fc.chan17_raw, fc.chan18_raw};
    rc_channels_ = channels;
}

void FcRcStatusReceiver::on_sys_status(const mavlink_message_t& message) {
    mavlink_sys_status_t fc{};
    mavlink_msg_sys_status_decode(&message, &fc);
    FcSensorHealth health;
    health.received_monotonic_ns = link_.on_fc_message();
    health.fc_epoch = link_.fc_epoch();
    health.rc_receiver_ok = present_and_healthy(fc, MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
    health.local_position_ok = present_and_healthy(fc, MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
    // PX4 는 자세 추정의 present 비트를 켜지 않으므로 health 비트만 본다
    health.attitude_ok = (fc.onboard_control_sensors_health & MAV_SYS_STATUS_AHRS) != 0;
    sensor_health_ = health;
}

}  // namespace argus::mavlink_gateway
