// 게이트웨이 2단계 단위 시험: MAVLink 코덱 왕복, 송신자별 시퀀스, UDP 링크(loopback), FlightLink
// 기본 거부. Python 원본과의 대량 비교는 tests/gateway/differential/stage2_diff.py가 맡는다.
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "argus/gateway/mavlink/include/mavlink_codec.hpp"
#include "argus/gateway/mavlink/include/mavlink_link.hpp"
#include "argus/gateway/vehicle/include/flight_link.hpp"

namespace gw = argus::gateway;
using nlohmann::json;

namespace {

int& failures() {
    static int count = 0;
    return count;
}

void check(bool condition, const std::string& what) {
    if (!condition) {
        ++failures();
        std::cerr << "실패: " << what << '\n';
    }
}

void codec_round_trip() {
    gw::MavlinkSender sender(245, 191);
    gw::MavlinkParser parser;
    parser.feed(sender.command_long(1, 1, 400, 0, {1, 0, 0, 0, 0, 0, 0}));
    parser.feed(sender.param_set(1, 1, "SYS_HAS_GPS", 1.5F, 9));
    parser.feed(sender.request_event(1, 1, 65530, 4));
    const auto command = parser.next();
    check(command && command->kind == "COMMAND_LONG" && command->system == 245 &&
              command->component == 191,
          "COMMAND_LONG 해석");
    if (command) {
        check(command->data.at("command") == 400 && command->data.at("param1") == 1.0,
              "COMMAND_LONG 값");
        check(command->data.at("mavpackettype") == "COMMAND_LONG", "mavpackettype");
    }
    const auto param = parser.next();
    check(param && param->data.at("param_id") == "SYS_HAS_GPS" &&
              param->data.at("param_value") == 1.5,
          "PARAM_SET 이름·값");
    const auto event = parser.next();
    check(
        event && event->data.at("first_sequence") == 65530 && event->data.at("last_sequence") == 4,
        "REQUEST_EVENT 순번");
    check(!parser.next().has_value(), "남은 메시지 없음");

    // 송신자마다 자기 시퀀스 (2번째 프레임의 seq는 1)
    gw::MavlinkSender other(244, 190);
    const gw::Bytes first = other.heartbeat(6, 8, 0, 0, 4);
    const gw::Bytes second = other.heartbeat(6, 8, 0, 0, 4);
    check(first.size() > 4 && first.at(4) == 0 && second.at(4) == 1, "송신자별 seq");

    // CRC가 틀린 프레임은 버린다
    gw::Bytes broken = sender.heartbeat(18, 8, 0, 0, 4);
    broken.back() ^= 0xffU;
    gw::MavlinkParser strict;
    strict.feed(broken);
    check(!strict.next().has_value(), "CRC 오류 프레임 버림");
}

void udp_loopback() {
    const int port = 20000 + (::getpid() % 20000);
    auto server = gw::open_link("udpin:127.0.0.1:" + std::to_string(port), 57600);
    auto client = gw::open_link("udpout:127.0.0.1:" + std::to_string(port), 57600);
    client->write(gw::Bytes{1, 2, 3});
    gw::Bytes received;
    for (int i = 0; i < 50 && received.empty(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        received = server->read_available();
    }
    check(received == gw::Bytes({1, 2, 3}), "udpout → udpin");
    server->write(gw::Bytes{9});  // udpin은 마지막 송신자에게 답한다
    gw::Bytes reply;
    for (int i = 0; i < 50 && reply.empty(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        reply = client->read_available();
    }
    check(reply == gw::Bytes({9}), "udpin → 마지막 송신자");
}

gw::FlightLinkOptions options(const std::string& uid) {
    gw::FlightLinkOptions value;
    value.expected_uid = uid;
    value.clock = [] { return 100.0; };
    value.event_metadata = "/nonexistent/fc-events.json";
    value.link_factory = [](const std::string&, int) -> std::unique_ptr<gw::ILink> {
        return nullptr;
    };
    return value;
}

void flight_link_guards() {
    bool rejected = false;
    try {
        const gw::FlightLink bad(options("xyz"));
    } catch (const gw::ValueError& error) {
        rejected = std::string(error.what()) == "FC UID는 18바이트 hex여야 합니다";
    }
    check(rejected, "잘못된 UID 거부");
    gw::FlightLink link(options("000600000000353733323333511700280027"));
    const auto expect = [&](auto action, const std::string& message, const std::string& what) {
        try {
            action();
            check(false, what + " (예외 없음)");
        } catch (const gw::ValueError& error) {
            check(error.what() == message, what + " → " + error.what());
        }
    };
    expect([&] { link.submit("fly", "gui", nullptr); }, "지원하지 않는 명령", "모르는 명령");
    expect([&] { link.submit("arm", "", nullptr); }, "관제 세션 ID 오류", "빈 세션");
    expect([&] { link.submit("arm", "gui", nullptr); }, "FC 미연결 또는 UID 확인 미완료/불일치",
           "미연결 ARM");
    expect([&] { link.request_connect("COM3", 57600, "gui"); },
           "/dev 또는 COM 직렬 장치, udpin:/udpout:host:port를 입력하세요", "주소 형식");
    expect([&] { link.request_connect("udpout:127.0.0.1:70000", 57600, "gui"); },
           "UDP 포트 범위 오류", "UDP 포트");
    expect([&] { link.request_connect("/dev/ttyACM0", 9600, "gui"); }, "지원하지 않는 baud rate",
           "baud");
    const json state = link.snapshot();
    check(state.at("linked") == false && state.at("controls").at("land") == false,
          "미연결 snapshot");
    check(state.at("reasons").at("arm") == "FC 미연결 또는 UID 확인 미완료/불일치", "미연결 사유");
}

// 2026-11-16: hold_view()가 snapshot()의 같은 키와 같은 값을 내는지 여러 FC 상태에서 비교한다.
void hold_view_matches_snapshot() {
    gw::FlightLink link(options("000600000000353733323333511700280027"));
    const auto compare = [&](const std::string& what) {
        const json view = link.hold_view();
        const json full = link.snapshot();
        for (const char* key : {"fresh", "identity_ok", "mode", "armed", "ground", "local_ok",
                                "height_m", "height_age_ms", "flight_environment"}) {
            check(view.at(key) == full.at(key),
                  what + " " + key + " " + view.at(key).dump() + " vs " + full.at(key).dump());
        }
        check(view.size() == 9, what + " 키 수");
    };
    compare("미연결");
    auto& state = link.state();
    state.uid = "000600000000353733323333511700280027";
    const auto put = [&](const char* kind, json data, double at) {
        state.samples[kind] = gw::Sample{std::move(data), at};
    };
    const int modes[] = {0x03040000, 0x04040000, 0x00030000,
                         0x00060000};  // HOLD·임무·POSCTL·OFFBOARD
    int case_index = 0;
    for (const double heartbeat_at : {99.9, 90.0}) {
        for (const int base_mode : {0x81, 0x01}) {
            for (const int custom_mode : modes) {
                for (const int landed : {1, 2}) {
                    for (const double local_at : {99.95, 99.0}) {
                        put("HEARTBEAT",
                            {{"type", 2},
                             {"autopilot", 12},
                             {"base_mode", base_mode},
                             {"custom_mode", custom_mode},
                             {"system_status", 4},
                             {"mavlink_version", 3}},
                            heartbeat_at);
                        put("EXTENDED_SYS_STATE", {{"vtol_state", 0}, {"landed_state", landed}},
                            99.95);
                        put("ESTIMATOR_STATUS",
                            {{"flags", case_index % 3 == 0 ? 0 : 0x0BFF}, {"time_usec", 1}}, 99.95);
                        put("LOCAL_POSITION_NED",
                            {{"time_boot_ms", 1000 + case_index},
                             {"x", 0.1},
                             {"y", -0.2},
                             {"z", -1.5},
                             {"vx", 0.0},
                             {"vy", 0.0},
                             {"vz", 0.0}},
                            local_at);
                        put("DISTANCE_SENSOR",
                            {{"time_boot_ms", 1000},
                             {"current_distance", 150 + case_index},
                             {"min_distance", 8},
                             {"max_distance", 1200},
                             {"orientation", 25}},
                            99.97);
                        compare("상태" + std::to_string(case_index++));
                    }
                }
            }
        }
    }
}

}  // namespace

int main() {
    try {
        codec_round_trip();
        udp_loopback();
        flight_link_guards();
        hold_view_matches_snapshot();
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures() != 0) {
        std::cerr << failures() << "개 실패\n";
        return 1;
    }
    std::cout << "GatewayStage2Test 통과\n";
    return 0;
}
