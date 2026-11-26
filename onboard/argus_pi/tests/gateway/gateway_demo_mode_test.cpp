// 2026-11-18 실내 데모 모드 시험: TRACK_START 회전(yaw) 모드 경로, 이륙/고도 변경 1.0~2.0 m.
// 가짜 B 추종기 응답과 FC 없는 FlightLink만 쓴다 (소켓, FC, 카메라는 열지 않음).
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "argus/gateway/bridge/include/follow_preferences.hpp"
#include "argus/gateway/bridge/include/track_adapter.hpp"
#include "argus/gateway/vehicle/include/flight_link.hpp"

namespace gw = argus::gateway;

namespace {

int failures = 0;

void check(bool value, const std::string& message) {
    if (!value) {
        ++failures;
        std::cerr << "FAIL " << message << '\n';
    }
}

gw::Wire controller_status(const char* controller, const char* mode) {
    gw::Wire status = gw::Wire::object();
    for (const char* key : {"accepted", "tracks_fresh", "input_contract_v5", "identity_ok",
                            "connected", "armed", "in_air", "actual_hold"}) {
        gw::wire_set(status, key, true);
    }
    gw::wire_set(status, "active", false);
    gw::wire_set(status, "bench_active", false);
    gw::wire_set(status, "mode", mode);
    gw::wire_set(status, "configured_follow_environment", "indoor");
    gw::wire_set(status, "configured_follow_controller", controller);
    gw::Wire candidate = gw::Wire::object();
    gw::wire_set(candidate, "sensor", "ir_sdk:0");
    gw::wire_set(candidate, "track_id", 7);
    gw::wire_set(candidate, "generation", 3);
    gw::wire_set(candidate, "class_id", 0);
    gw::wire_set(candidate, "label", "person");
    gw::wire_set(status, "candidates", gw::Wire::array({candidate}));
    return status;
}

struct FakeController {
    std::string controller;
    std::string mode;
    bool mode_change_fails = false;
    std::vector<std::string> calls;

    gw::Wire operator()(const gw::Wire& body, const std::string& /*uid*/) {
        const std::string action = body.at("action").get<std::string>();
        std::string record = action;
        if (action == "mode") {
            record += ":" + body.at("mode").get<std::string>();
            if (!mode_change_fails) {
                mode = body.at("mode").get<std::string>();
            }
        }
        calls.push_back(record);
        if (action == "status") {
            return controller_status(controller.c_str(), mode.c_str());
        }
        gw::Wire reply = gw::Wire::object();
        gw::wire_set(reply, "accepted", true);
        if (action == "start") {
            gw::wire_set(reply, "controller_session", "c");
            gw::wire_set(reply, "onboard_session", "o");
            gw::wire_set(reply, "fc_epoch", 1);
            gw::wire_set(reply, "pending", true);
        }
        return reply;
    }
};

std::vector<std::string> run_start(FakeController& fake, std::string* error = nullptr) {
    const gw::Wire params = gw::Wire::array({7, 1, 3, 0});
    try {
        (void)gw::track_start(
            params, "uid",
            [&fake](const gw::Wire& body, const std::string& uid) { return fake(body, uid); },
            gw::Admission(), [] { return std::string("req"); });
    } catch (const gw::BridgeRejected& rejected) {
        if (error != nullptr) {
            *error = rejected.what();
        }
    }
    return fake.calls;
}

using Calls = std::vector<std::string>;

void track_start_demo_test() {
    {
        FakeController fake{"demo", "moving"};
        check(run_start(fake) == Calls{"status", "prepare_handoff", "mode:yaw", "status", "start"},
              "demo from moving: request yaw, verify, start");
    }
    {
        FakeController fake{"demo", "yaw"};
        check(run_start(fake) == Calls{"status", "start"}, "demo already yaw with HOLD: start");
    }
    {
        FakeController fake{"demo", "moving", true};
        std::string error;
        check(run_start(fake, &error) == Calls{"status", "prepare_handoff", "mode:yaw", "status"},
              "demo yaw not reached: no start");
        check(error == "데모 모드 회전(yaw) 모드 또는 최신 대상 확인 실패",
              "demo yaw failure message");
    }
    {
        FakeController fake{"predictive", "yaw"};
        check(
            run_start(fake) == Calls{"status", "prepare_handoff", "mode:moving", "status", "start"},
            "predictive unchanged: request moving");
    }
    {
        FakeController fake{"predictive", "moving"};
        check(run_start(fake) == Calls{"status", "start"}, "predictive already moving");
    }
    {
        FakeController fake{"predictive", "yaw", true};
        std::string error;
        (void)run_start(fake, &error);
        check(error == "실제 이동 모드 또는 최신 대상 확인 실패", "predictive failure message");
    }
    // 2026-11-19 데모2는 예측 이동 추종이라 기본처럼 이동(moving) 모드에서 시작한다.
    {
        FakeController fake{"demo2", "yaw"};
        check(
            run_start(fake) == Calls{"status", "prepare_handoff", "mode:moving", "status", "start"},
            "demo2: request moving");
    }
    {
        FakeController fake{"demo2", "moving"};
        check(run_start(fake) == Calls{"status", "start"}, "demo2 already moving");
    }
    std::cout << "PASS demo track start\n";
}

gw::FlightLinkOptions options() {
    gw::FlightLinkOptions value;
    value.expected_uid = "000600000000353733323333511700280027";
    value.clock = [] { return 100.0; };
    value.event_metadata = "/nonexistent/fc-events.json";
    value.link_factory = [](const std::string&, int) -> std::unique_ptr<gw::ILink> {
        return nullptr;
    };
    return value;
}

std::string height_error(gw::FlightLink& link, const gw::Json& height) {
    const std::scoped_lock lock(link.mutex());
    try {
        (void)link.validate_height(height);
    } catch (const gw::ValueError& error) {
        return error.what();
    }
    return "";
}

void height_band_test() {
    gw::FlightLink link(options());
    {
        const std::scoped_lock lock(link.mutex());
        link.state().params["SENS_FLOW_MAXHGT"] = 5.0;
        link.state().samples["DISTANCE_SENSOR"] =
            gw::Sample{gw::Json{{"max_distance", 1200}}, 100.0};
    }
    // 기본: 기존 한계 0.5~4.8 m
    check(height_error(link, 2.5).empty(), "no band: 2.5 m allowed");
    check(height_error(link, 0.6).empty(), "no band: 0.6 m allowed");
    check(!height_error(link, 0.2).empty(), "no band: below minimum rejected");
    bool demo = true;
    link.set_height_band_source([&demo]() -> std::optional<gw::FlightLink::HeightBand> {
        if (!demo) {
            return std::nullopt;
        }
        return gw::FlightLink::HeightBand{gw::kDemoMinHeightM, gw::kDemoMaxHeightM,
                                          gw::kDemoHeightMessage};
    });
    const std::string message = gw::kDemoHeightMessage;
    for (const double height : {1.0, 1.5, 2.0}) {
        check(height_error(link, height).empty(), "demo: " + std::to_string(height) + " allowed");
    }
    for (const double height : {0.6, 0.99, 2.01, 2.5, 4.0, 0.2}) {
        check(height_error(link, height) == message,
              "demo: " + std::to_string(height) + " rejected with demo message");
    }
    check(height_error(link, "abc") != message && !height_error(link, "abc").empty(),
          "demo: non-number keeps existing message");
    {
        const std::scoped_lock lock(link.mutex());
        link.state().params["SENS_FLOW_MAXHGT"] = 1.8;
    }
    check(!height_error(link, 1.9).empty() && height_error(link, 1.9) != message,
          "demo: existing sensor limit still applies");
    {
        const std::scoped_lock lock(link.mutex());
        link.state().params["SENS_FLOW_MAXHGT"] = 5.0;
    }
    // submit도 같은 검사를 지난다 (FC 미연결이라도 높이 검사가 먼저)
    try {
        (void)link.submit("takeoff", "gui", 2.5);
        check(false, "demo submit takeoff 2.5 m rejected");
    } catch (const gw::ValueError& error) {
        check(error.what() == message, std::string("demo submit message: ") + error.what());
    }
    try {
        (void)link.submit("set_height", "gui", 0.8);
        check(false, "demo submit set_height 0.8 m rejected");
    } catch (const gw::ValueError& error) {
        check(error.what() == message, std::string("demo set_height message: ") + error.what());
    }
    demo = false;
    check(height_error(link, 2.5).empty(), "band off: 2.5 m allowed again");
    link.set_height_band_source({});
    {
        const std::scoped_lock lock(link.mutex());
        check(!link.height_band(), "cleared source: no band");
    }
    std::cout << "PASS demo height band\n";
}

}  // namespace

int main() {
    try {
        track_start_demo_test();
        height_band_test();
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures != 0) {
        std::cerr << failures << "개 실패\n";
        return 1;
    }
    return 0;
}
