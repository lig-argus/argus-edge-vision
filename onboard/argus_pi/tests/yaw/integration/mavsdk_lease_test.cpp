// 설치된 MAVSDK의 실제 C API hook을 raw:// 모의 FC로 검사한다. UART/UDP/실기체는 열지 않는다.
#include <unistd.h>

#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "argus/vehicle/mavsdk/include/file_command_owner.hpp"
#include "argus/vehicle/mavsdk/include/mavsdk_identity_guard.hpp"
#include "argus/vehicle/mavsdk/include/mavsdk_vehicle_port.hpp"
#include "argus/vehicle/mavsdk/include/sdk_headers.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {
struct Record {
    Clock::time_point at;
    float yaw;
};
struct Output {
    std::mutex mutex;
    std::vector<Record> records;
};

template <class Predicate>
bool wait_for(Predicate predicate, std::chrono::milliseconds timeout = 2000ms) {
    const auto until = Clock::now() + timeout;
    do {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(10ms);
    } while (Clock::now() < until);
    return predicate();
}

void receive(mavsdk::Mavsdk& sdk, const mavlink_message_t& message) {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
    const auto count = mavlink_msg_to_send_buffer(bytes.data(), &message);
    sdk.pass_received_raw_bytes(reinterpret_cast<const char*>(bytes.data()), count);
}

std::vector<Record> records(const std::shared_ptr<Output>& output) {
    std::lock_guard<std::mutex> lock(output->mutex);
    return output->records;
}
}  // namespace

int main() {
    mavsdk::Mavsdk sdk{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::CompanionComputer}};
    auto output = std::make_shared<Output>();
    const auto subscription =
        sdk.subscribe_raw_bytes_to_be_sent([output](const char* data, std::size_t length) {
            std::lock_guard<std::mutex> lock(output->mutex);
            mavlink_message_t message{};
            mavlink_status_t status{};
            for (std::size_t i = 0; i < length; ++i) {
                if (!mavlink_parse_char(MAVLINK_COMM_0, static_cast<std::uint8_t>(data[i]),
                                        &message, &status)) {
                    continue;
                }
                if (message.msgid != MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
                    continue;
                }
                mavlink_set_position_target_local_ned_t packet{};
                mavlink_msg_set_position_target_local_ned_decode(&message, &packet);
                assert(packet.vx == 0 && packet.vy == 0 && packet.vz == 0);
                output->records.push_back({Clock::now(), packet.yaw_rate});
            }
        });
    assert(sdk.add_any_connection("raw://") == mavsdk::ConnectionResult::Success);
    constexpr char uid[] = "000600000000353733323333511700280027";
    MavsdkIdentityGuard identity{sdk, uid, 1, 1};
    std::atomic<bool> running{true}, correct_uid{false};
    std::thread beacon([&] {
        while (running) {
            mavlink_message_t heartbeat{};
            mavlink_msg_heartbeat_pack(1, 1, &heartbeat, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                                       MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_STANDBY);
            receive(sdk, heartbeat);
            mavlink_autopilot_version_t version{};
            version.vendor_id = 1;
            version.product_id = 1;
            version.flight_sw_version = 0x010f0000;
            for (std::size_t i = 0; i < sizeof(version.uid2); ++i) {
                const auto nibble = [](char c) { return c >= 'a' ? c - 'a' + 10 : c - '0'; };
                version.uid2[i] =
                    static_cast<std::uint8_t>((nibble(uid[2 * i]) << 4) | nibble(uid[2 * i + 1]));
            }
            if (!correct_uid) {
                version.uid2[0] ^= 0xff;
            }
            mavlink_message_t identification{};
            mavlink_msg_autopilot_version_encode(1, 1, &identification, &version);
            receive(sdk, identification);
            std::this_thread::sleep_for(30ms);
        }
    });
    const auto system = sdk.first_autopilot(3);
    assert(system);
    assert(!identity.verify(*system));
    correct_uid = true;
    assert(wait_for([&] { return identity.writer_permit()(); }));
    assert(identity.verify(*system));
    {
        mavsdk::Offboard offboard{*system};
        const auto owner_path = std::filesystem::temp_directory_path() /
                                ("thessen-sdk-owner-" + std::to_string(::getpid()));
        const auto owner_name = owner_path.string();
        FileCommandOwner owner{owner_name.c_str()};
        const auto permit = identity.writer_permit();
        MavsdkVehiclePort port{offboard, sdk, "lease-test",
                               [&] { return permit() && owner.owns(); }};
        FollowCommand command;
        command.yaw_rate_deg_s = 6;
        assert(!port.send_until(command, Clock::now() + 500ms));
        assert(owner.acquire());
        const auto deadline = Clock::now() + 600ms;
        assert(port.send_until(command, deadline));
        assert(wait_for([&] { return !records(output).empty(); }));
        std::this_thread::sleep_until(deadline + 120ms);
        const auto sent = records(output);
        assert(!sent.empty());
        for (const auto& record : sent) {
            assert(record.at < deadline);
        }
        const auto expired_count = sent.size();
        std::this_thread::sleep_for(150ms);
        assert(records(output).size() == expired_count);

        // SDK를 직접 호출한 다른 setpoint도 보관된 후보와 다르면 출구에서 차단한다.
        assert(port.send_until(command, Clock::now() + 500ms));
        assert(offboard.set_velocity_body({0, 0, 0, 9}) == mavsdk::Offboard::Result::Success);
        std::this_thread::sleep_for(120ms);
        for (const auto& record : records(output)) {
            assert(std::abs(record.yaw - 9 * std::acos(-1.) / 180) > 1e-5);
        }

        correct_uid = false;
        assert(wait_for([&] { return !identity.writer_permit()(); }));
        assert(!port.send_until(command, Clock::now() + 500ms));
        const auto denied_count = records(output).size();
        std::this_thread::sleep_for(150ms);
        assert(records(output).size() == denied_count);

        correct_uid = true;
        assert(wait_for([&] { return identity.writer_permit()(); }));
        const auto zero_count = records(output).size();
        assert(port.send_until(FollowCommand{}, Clock::now() + 500ms));
        assert(wait_for([&] { return records(output).size() > zero_count; }));
        port.revoke_commands();
        std::this_thread::sleep_for(60ms);
        const auto revoked_count = records(output).size();
        std::this_thread::sleep_for(150ms);
        assert(records(output).size() == revoked_count);
        assert(port.send_until(command, Clock::now() + 500ms));
        owner.release();
        const auto ownership_lost_count = records(output).size();
        std::this_thread::sleep_for(180ms);
        assert(records(output).size() == ownership_lost_count);
        assert(!port.send_until(command, Clock::now() + 500ms));
        port.revoke_commands();
        std::filesystem::remove(owner_path);
    }
    running = false;
    beacon.join();
    sdk.unsubscribe_raw_bytes_to_be_sent(subscription);
    std::cout
        << "installed SDK identity/TTL/mismatched setpoint/revoke guards passed; raw mock only\n";
}
