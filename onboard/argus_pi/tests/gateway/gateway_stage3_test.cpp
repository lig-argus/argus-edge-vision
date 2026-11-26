#include <common/mavlink.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "argus/gateway/follow/include/follow_owner.hpp"
#include "argus/gateway/vehicle/include/rc_reclaim.hpp"

namespace gw = argus::gateway;
using namespace std::chrono_literals;
namespace {
void require(bool value, const char* message) {
    if (!value) {
        throw std::runtime_error(message);
    }
}
template <typename F>
void rejects(const F& action) {
    try {
        action();
    } catch (const gw::ValueError&) {
        return;
    }
    throw std::runtime_error("expected rejection");
}

void authority_test() {
    gw::OperatorAuthority owner;
    require(owner.admit("main:a") == 1, "initial admission");
    require(owner.admit("main:a") == 1, "idempotent admission");
    rejects([&] { owner.admit("main:b"); });
    // 2026-11-16: 살아 있는 다른 GUI가 있어도(비행 중) 명시적 가져오기는 허용, 추종·FC 처리 중은
    // 거절
    rejects([&] { owner.main_gui_claim("main:b", true, true, false, false); });
    rejects([&] { owner.main_gui_claim("main:b", true, false, true, false); });
    require(owner.owner() == "main:a", "rejected claim keeps owner");
    require(owner.main_gui_claim("main:b", true, false, false, false) == "main:a",
            "explicit claim from live operator");
    owner.set_emergency(true);
    require(owner.admit("main:a", true) == 2, "emergency admission without ownership");
    rejects([&] { owner.maintenance_claim("maintenance"); });
    rejects([&] { owner.main_gui_claim("main:a", false, false, false, false); });
    owner.main_gui_claim("main:a", true, false, false, true);
    require(!owner.emergency() && owner.owner() == "main:a", "ground latch clear");
    rejects([&] { owner.release("main:a", false); });
    owner.release("main:a", true);
    require(!owner.owner() && owner.snapshot().at("revision") == 4, "inactive release");
}

gw::Json calibration() {
    gw::Json result = {
        {"RC_MAP_ROLL", 1}, {"RC_MAP_PITCH", 2}, {"RC_MAP_YAW", 3}, {"RC_MAP_THROTTLE", 4}};
    for (int channel = 1; channel <= 4; ++channel) {
        const auto prefix = "RC" + std::to_string(channel);
        gw::json_set(result, prefix + "_MIN", 1000);
        gw::json_set(result, prefix + "_MAX", 2000);
        gw::json_set(result, prefix + "_TRIM", channel == 4 ? 1000 : 1500);
        gw::json_set(result, prefix + "_DZ", 25);
        gw::json_set(result, prefix + "_REV", 1);
    }
    return result;
}
gw::Json channels(bool moved = false) {
    return {{"chan1_raw", moved ? 1600 : 1500},
            {"chan2_raw", 1500},
            {"chan3_raw", 1500},
            {"chan4_raw", 1500}};
}

void neutral_test() {
    gw::NeutralObservation neutral;
    const auto params = calibration();
    for (int i = 0; i <= 4; ++i) {
        const double t = i * .25;
        require(neutral.update(t, t, channels(), params, true).at("ready") == (i == 4),
                "neutral dwell");
    }
    require(neutral.update(1.01, 1.01, channels(true), params, true).at("ready") == false,
            "stick movement invalidates");
    require(neutral.update(2, 1.5, channels(), params, true).at("ready") == false,
            "stale sample invalidates");
    // 2026-11-09 중립 허용은 반행정(500)의 15% = 75µs. DZ(25)보다 넓다. 스로틀도 중간값 기준.
    const auto within = [&](int roll, int throttle) {
        gw::NeutralObservation wide;
        gw::Json values = channels();
        values["chan1_raw"] = roll;
        values["chan4_raw"] = throttle;
        bool ready = false;
        for (int i = 0; i <= 4; ++i) {
            const double t = i * .25;
            ready = wide.update(t, t, values, params, true).at("ready").get<bool>();
        }
        return ready;
    };
    require(within(1570, 1500), "14% roll offset is neutral");
    require(within(1425, 1500), "15% roll offset is neutral");
    require(!within(1580, 1500), "16% roll offset is not neutral");
    require(within(1500, 1430), "14% throttle offset from middle is neutral");
    require(!within(1500, 1420), "16% throttle offset from middle is not neutral");
}

class FakeLink final : public gw::ILink {
public:
    gw::Bytes incoming;
    gw::MavlinkParser sent;
    gw::Bytes read_available() override { return std::exchange(incoming, {}); }
    void write(const gw::Bytes& data) override { sent.feed(data); }
    void close() override {}
};

void inject_calibration(FakeLink& link, const std::string& only = "",
                        const std::string& skip = "") {
    mavlink_status_t status{};
    const auto params = calibration();
    for (auto it = params.begin(); it != params.end(); ++it) {
        if ((!only.empty() && it.key() != only) || it.key() == skip) {
            continue;
        }
        std::array<char, 16> name{};
        std::copy(it.key().begin(), it.key().end(), name.begin());
        mavlink_message_t frame{};
        mavlink_msg_param_value_pack_status(1, 1, &status, &frame, name.data(), it->get<float>(),
                                            MAV_PARAM_TYPE_REAL32, 20, 0);
        std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
        const auto size = mavlink_msg_to_send_buffer(bytes.data(), &frame);
        link.incoming.insert(link.incoming.end(), bytes.begin(), std::next(bytes.begin(), size));
    }
}

void seed(gw::FlightState& state, double now, const std::string& mode, bool moved = false) {
    const auto set = [&](const char* name, gw::Json data) {
        state.samples.insert_or_assign(name, gw::Sample{std::move(data), now});
    };
    const auto numbers = gw::mode_numbers(mode).value_or(std::pair<int, int>{0, 0});
    set("HEARTBEAT",
        {{"base_mode", 128}, {"custom_mode", (numbers.first << 16) | (numbers.second << 24)}});
    set("EXTENDED_SYS_STATE", {{"landed_state", 2}});
    set("ESTIMATOR_STATUS", {{"flags", 79}, {"pos_horiz_accuracy", .1}});
    set("LOCAL_POSITION_NED", {{"x", 2}, {"y", 3}, {"z", -1.7}, {"vx", 0}, {"vy", 0}, {"vz", 0}});
    set("ATTITUDE", {{"roll", 0}, {"pitch", 0}, {"yaw", .2}});
    set("DISTANCE_SENSOR", {{"current_distance", 170}, {"min_distance", 8}, {"max_distance", 400}});
    set("OPTICAL_FLOW_RAD", {{"quality", 50}});
    set("SYS_STATUS", {{"onboard_control_sensors_present", 65539},
                       {"onboard_control_sensors_enabled", 65539},
                       {"onboard_control_sensors_health", 65539}});
    set("RC_CHANNELS", channels(moved));
}

template <typename F>
void wait_for(const F& condition, const char* reason) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!condition()) {
        require(std::chrono::steady_clock::now() < deadline, reason);
        std::this_thread::sleep_for(5ms);
    }
}

void reclaim_test(bool wrong_session) {
    gw::OperatorAuthority authority;
    authority.admit("main:test");
    std::atomic<double> now{100};
    std::string stop_id;
    gw::FlightLinkOptions options;
    options.expected_uid = "000600000000353733323333511700280027";
    options.event_metadata = "/nonexistent/fc-events.json";
    options.clock = [&] { return now.load(); };
    options.new_id = [] { return "test-command"; };
    gw::ReclaimFlightLink link(
        std::move(options),
        {[] { return true; },
         [&](const gw::Json& request, int ttl) -> gw::Json {
             require(ttl == 500, "bounded operator timeout");
             if (request.at("action") == "stop") {
                 stop_id = request.at("request_id").get<std::string>();
                 return {{"accepted", true},
                         {"pending", true},
                         {"controller_session", "controller"},
                         {"onboard_session", "onboard"},
                         {"fc_epoch", 0}};
             }
             return {{"accepted", true},
                     {"controller_session", wrong_session ? "changed" : "controller"},
                     {"onboard_session", "onboard"},
                     {"fc_epoch", 0},
                     {"command_results",
                      gw::Json::array(
                          {{{"command_id", stop_id}, {"pending", false}, {"accepted", true}}})}};
         }},
        &authority);
    auto& fc = link.state();
    auto transport = std::make_unique<FakeLink>();
    auto* mock = transport.get();
    inject_calibration(*mock);
    fc.link = std::move(transport);
    fc.mav = std::make_unique<gw::MavlinkSender>(245, 191);
    fc.uid = link.expected_uid();
    fc.config_requested = true;
    for (const auto& [key, val] : gw::indoor_profile()) {
        fc.params.insert_or_assign(key, val);
    }
    fc.params.insert_or_assign("EKF2_OF_QMIN", 1);
    fc.params.insert_or_assign("COM_POS_FS_EPH", 5);
    fc.params.insert_or_assign("SENS_FLOW_MAXHGT", 2);
    seed(fc, now, "position");
    rejects([&] { link.reclaim("main:test"); });
    for (int i = 0; i <= 4; ++i) {
        now = 100 + (i * .25);
        seed(fc, now, "position");
        link.step();
    }
    require(link.reclaim("main:test").at("accepted") == true, "reclaim accepted");
    wait_for(
        [&] {
            const std::scoped_lock lock(link.mutex());
            return !fc.jobs.empty();
        },
        "HOLD queued");
    link.step();
    {
        const std::scoped_lock lock(link.mutex());
        require(fc.pending && fc.pending->at("action") == "hold", "HOLD issued first");
        require(!fc.local_target, "position priming waits for HOLD");
        now = 101.05;
        seed(fc, now, "hold");
        gw::json_set(*fc.pending, "ack", true);
        gw::json_set(*fc.pending, "ack_at", now.load());
    }
    link.step();
    wait_for(
        [&] {
            const std::scoped_lock lock(link.mutex());
            if (wrong_session) {
                return fc.last_job && fc.last_job->value("status", std::string()) == "failed";
            }
            return !fc.jobs.empty();
        },
        "controller STOP terminal confirmation");
    link.step();
    if (wrong_session) {
        require(!fc.pending && !fc.local_target, "foreign STOP result blocks priming");
        return;
    }
    require(fc.pending && fc.pending->at("stage") == "priming" && fc.local_target &&
                fc.local_target->at("x") == 2 && fc.local_target->at("target_z") == -1.7,
            "position captured only after real HOLD and STOP completion");
    now = 101.1;
    seed(fc, now, "hold", true);
    link.step();
    require(!fc.pending && !fc.local_target && !fc.owned_flight, "stick movement revokes priming");
    while (const auto packet = mock->sent.next()) {
        if (packet->kind == "COMMAND_LONG") {
            require(packet->data.at("command") != 400, "no ARM commands");
        }
    }
}

// 2026-11-10 실내 RC→GUI 복귀: HOLD를 보내지 않고 현재 위치 Offboard 제자리 유지를 먼저 잡은 뒤
// 추종기 STOP을 확인한다.
void indoor_offboard_reclaim_test() {
    gw::OperatorAuthority authority;
    authority.admit("main:test");
    std::atomic<double> now{100};
    std::string stop_id;
    std::atomic<int> stops{0};
    gw::FlightLinkOptions options;
    options.expected_uid = "000600000000353733323333511700280027";
    options.event_metadata = "/nonexistent/fc-events.json";
    options.clock = [&] { return now.load(); };
    options.new_id = [] { return "test-command"; };
    gw::ReclaimFlightLink link(
        std::move(options),
        {[] { return true; },
         [&](const gw::Json& request, int) -> gw::Json {
             if (request.at("action") == "stop") {
                 ++stops;
                 stop_id = request.at("request_id").get<std::string>();
                 return {{"accepted", true},
                         {"pending", true},
                         {"controller_session", "controller"},
                         {"onboard_session", "onboard"},
                         {"fc_epoch", 0}};
             }
             return {{"accepted", true},
                     {"controller_session", "controller"},
                     {"onboard_session", "onboard"},
                     {"fc_epoch", 0},
                     {"command_results",
                      gw::Json::array(
                          {{{"command_id", stop_id}, {"pending", false}, {"accepted", true}}})}};
         }},
        &authority);
    link.set_indoor_offboard_reclaim(true);
    auto& fc = link.state();
    auto transport = std::make_unique<FakeLink>();
    auto* mock = transport.get();
    inject_calibration(*mock);
    fc.link = std::move(transport);
    fc.mav = std::make_unique<gw::MavlinkSender>(245, 191);
    fc.uid = link.expected_uid();
    fc.config_requested = true;
    for (const auto& [key, val] : gw::indoor_profile()) {
        fc.params.insert_or_assign(key, val);
    }
    fc.params.insert_or_assign("EKF2_OF_QMIN", 1);
    fc.params.insert_or_assign("COM_POS_FS_EPH", 5);
    fc.params.insert_or_assign("SENS_FLOW_MAXHGT", 2);
    for (int i = 0; i <= 4; ++i) {
        now = 100 + (i * .25);
        seed(fc, now, "position");
        link.step();
    }
    require(link.environment() == std::optional<std::string>("indoor"), "indoor profile");
    require(link.reclaim("main:test").at("accepted") == true, "indoor reclaim accepted");
    wait_for(
        [&] {
            const std::scoped_lock lock(link.mutex());
            return !fc.jobs.empty();
        },
        "offboard reclaim queued");
    link.step();
    {
        const std::scoped_lock lock(link.mutex());
        require(fc.pending && fc.pending->at("action") == "offboard" &&
                    fc.pending->value("reclaim", false),
                "indoor: Offboard reclaim first, no HOLD");
        require(fc.pending->at("stage") == "priming" && fc.local_target &&
                    fc.local_target->at("x") == 2 && fc.local_target->at("target_z") == -1.7,
                "current position captured");
    }
    // priming 1.3초 뒤 Offboard 모드 명령 → FC가 Offboard로 바뀜
    bool commanded = false;
    for (int i = 1; i <= 30 && !commanded; ++i) {
        const std::scoped_lock lock(link.mutex());
        now = 101 + (i * .1);
        seed(fc, now, "position");
        link.step();
        commanded = fc.pending && fc.pending->at("stage") == "command";
    }
    require(commanded, "offboard mode command after priming");
    {
        const std::scoped_lock lock(link.mutex());
        now = now + .05;
        seed(fc, now, "offboard");
        gw::json_set(*fc.pending, "ack", true);
        gw::json_set(*fc.pending, "ack_at", now.load());
    }
    for (int i = 0; i < 3; ++i) {
        const std::scoped_lock lock(link.mutex());
        now = now + .05;
        seed(fc, now, "offboard");
        link.step();
    }
    wait_for(
        [&] {
            const std::scoped_lock lock(link.mutex());
            now = now + .01;
            seed(fc, now, "offboard");
            link.step();
            return fc.last_job && fc.last_job->value("action", std::string()) == "rc_gui_handoff";
        },
        "indoor reclaim completes");
    {
        const std::scoped_lock lock(link.mutex());
        require(fc.last_job->value("status", std::string()) == "done",
                fc.last_job->value("message", std::string("indoor reclaim done")).c_str());
        require(stops.load() >= 1, "controller STOP confirmed after Offboard hold");
        require(fc.owned_flight && fc.local_target, "GUI keeps current-position hold");
    }
    // HOLD(AUTO.LOITER, main 4 sub 3) 모드 명령은 보내지 않았다
    while (const auto packet = mock->sent.next()) {
        if (packet->kind == "COMMAND_LONG" && packet->data.at("command") == 176) {
            require(!(packet->data.at("param2") == 4 && packet->data.at("param3") == 3),
                    "no HOLD mode request indoors");
        }
        if (packet->kind == "COMMAND_LONG") {
            require(packet->data.at("command") != 400, "no ARM commands");
        }
    }
}

std::vector<std::string> requested_params(FakeLink& link) {
    std::vector<std::string> names;
    while (const auto packet = link.sent.next()) {
        if (packet->kind != "PARAM_REQUEST_READ") {
            continue;
        }
        const auto name = packet->data.value("param_id", std::string());
        if (name.rfind("RC_MAP_", 0) == 0 ||
            (name.size() > 2 && name.rfind("RC", 0) == 0 &&
             std::isdigit(static_cast<unsigned char>(name.at(2))) != 0)) {
            names.push_back(name);
        }
    }
    return names;
}

// 첫 일괄 응답에서 RC_MAP_PITCH가 빠져도 다시 요청해 RC 중립 판정이 가능해져야 한다.
void calibration_retry_test() {
    std::atomic<double> now{200};
    gw::FlightLinkOptions options;
    options.expected_uid = "000600000000353733323333511700280027";
    options.event_metadata = "/nonexistent/fc-events.json";
    options.clock = [&] { return now.load(); };
    options.new_id = [] { return "test-command"; };
    gw::ReclaimFlightLink link(
        std::move(options),
        {[] { return true; }, [](const gw::Json&, int) -> gw::Json { return {}; }}, nullptr);
    auto& fc = link.state();
    auto transport = std::make_unique<FakeLink>();
    auto* mock = transport.get();
    fc.link = std::move(transport);
    fc.mav = std::make_unique<gw::MavlinkSender>(245, 191);
    fc.uid = link.expected_uid();
    seed(fc, now, "position");
    link.step();
    require(fc.config_requested, "configure ran");
    const auto initial = requested_params(*mock);
    require(std::count(initial.begin(), initial.end(), "RC_MAP_PITCH") == 1,
            "initial burst requests RC_MAP_PITCH once");
    inject_calibration(*mock, "", "RC_MAP_PITCH");
    seed(fc, now, "position");
    link.step();
    require(requested_params(*mock).empty(), "no immediate retry");
    std::vector<std::string> retried;
    for (int i = 1; i <= 16; ++i) {
        now = 200 + (i * .25);
        seed(fc, now, "position");
        link.step();
        const auto names = requested_params(*mock);
        retried.insert(retried.end(), names.begin(), names.end());
    }
    require(std::count(retried.begin(), retried.end(), "RC_MAP_PITCH") >= 1,
            "missing RC_MAP_PITCH retried");
    for (const auto& name : retried) {
        require(name == "RC_MAP_PITCH", "only missing calibration retried");
    }
    require(retried.size() <= 3, "retry is rate limited");
    inject_calibration(*mock, "RC_MAP_PITCH");
    for (int i = 17; i <= 40; ++i) {
        now = 200 + (i * .25);
        seed(fc, now, "position");
        link.step();
        require(requested_params(*mock).empty(), "no retry once calibration complete");
    }
}
// 추종 소유자의 ZMQ 송신을 그대로 모은다 (2026-11-09 자세 즉시 송신 시험)
class CaptureTransport final : public gw::IFollowTransport {
public:
    explicit CaptureTransport(std::vector<gw::Parts>& sent) : sent_(sent) {}
    void publish(const gw::Parts& parts) override { sent_.push_back(parts); }
    std::optional<gw::Parts> rpc_receive() override { return std::nullopt; }
    void rpc_send(const gw::Parts& /*parts*/) override {}
    std::optional<gw::Parts> command_receive() override { return std::nullopt; }
    bool has_tracks() const override { return false; }
    std::optional<gw::Parts> track_receive() override { return std::nullopt; }
    void close() override {}

private:
    std::vector<gw::Parts>& sent_;
};

// 2026-11-09 실기: 30ms 송신 주기 꼬리 때문에 추적기가 프레임 뒤쪽 자세 표본을 늦게 받았다.
// 새 ATTITUDE는 받은 루프에서 바로 fc/attitude로 나가고, fc/state 주기는 그대로여야 한다.
void attitude_publish_test() {
    std::atomic<double> now{300};
    gw::FlightLinkOptions options;
    options.expected_uid = "000600000000353733323333511700280027";
    options.event_metadata = "/nonexistent/fc-events.json";
    options.clock = [&] { return now.load(); };
    options.new_id = [] { return "test-session"; };
    gw::FlightLink link(std::move(options));
    auto& fc = link.state();
    std::vector<gw::Parts> sent;
    fc.follow_b = std::make_unique<gw::FollowBOwner>(link, std::make_unique<CaptureTransport>(sent),
                                                     gw::FollowOwnerOptions{});
    seed(fc, now, "position");
    const auto topic_of = [](const gw::Parts& parts) {
        return std::string(parts.at(0).begin(), parts.at(0).end());
    };
    const auto stamp_of = [](const gw::Parts& parts) {
        return gw::Json::from_msgpack(parts.at(1)).at("received_monotonic_ns").get<std::int64_t>();
    };
    link.step();
    require(sent.size() == 2 && topic_of(sent.at(0)) == "fc/state" &&
                topic_of(sent.at(1)) == "fc/attitude",
            "first tick publishes state and attitude");
    int states = 0;
    int fresh_attitudes = 0;
    int updates = 0;
    std::int64_t newest = stamp_of(sent.at(1));
    double next_attitude = 300.017;
    for (int step = 1; step <= 30; ++step) {
        now = 300 + (step * .01);
        std::optional<std::int64_t> expected;
        if (now.load() >= next_attitude) {
            fc.samples.insert_or_assign(
                "ATTITUDE",
                gw::Sample{{{"roll", 0}, {"pitch", 0}, {"yaw", .2 + (step * .01)}}, now.load()});
            next_attitude += .017;
            expected = static_cast<std::int64_t>(now.load() * 1e9);
            ++updates;
        }
        sent.clear();
        link.step();
        bool delivered = false;
        for (const auto& parts : sent) {
            const std::string topic = topic_of(parts);
            if (topic == "fc/state") {
                ++states;
            } else if (topic == "fc/attitude") {
                const auto stamp = stamp_of(parts);
                require(stamp >= newest, "attitude stamps never go backwards");
                if (stamp > newest) {
                    ++fresh_attitudes;
                    newest = stamp;
                }
                delivered = delivered || (expected && stamp == *expected);
            }
        }
        require(!expected || delivered, "new attitude is published in the same loop");
        require(expected || sent.empty() || topic_of(sent.at(0)) == "fc/state",
                "no attitude without a new sample except the 30ms state publish");
    }
    require(fresh_attitudes == updates, "every new attitude sample is published once");
    require(states >= 6 && states <= 10, "fc/state keeps the 30ms cadence");
}

// 2026-11-24 사용자 지시: 100 Hz ATTITUDE와 10 ms 루프에서 한 루프에 표본 2개가 겹치면 앞의 것이
// fc/attitude로 나가지 않았다. 실제 ingest 경로로 이력을 채워, 마지막 송신 뒤 표본이 수신 순서대로
// 한 번씩 나가는지, 첫 송신이 지난 이력을 몰아 보내지 않는지, 한 번에 16개까지인지 확인한다.
class IngestFlightLink final : public gw::FlightLink {
public:
    using gw::FlightLink::FlightLink;
    using gw::FlightLink::ingest;
};

void attitude_burst_test() {
    std::atomic<double> now{299.96};
    gw::FlightLinkOptions options;
    options.expected_uid = "000600000000353733323333511700280027";
    options.event_metadata = "/nonexistent/fc-events.json";
    options.clock = [&] { return now.load(); };
    options.new_id = [] { return "test-session"; };
    IngestFlightLink link(std::move(options));
    auto& fc = link.state();
    std::vector<gw::Parts> sent;
    fc.follow_b = std::make_unique<gw::FollowBOwner>(link, std::make_unique<CaptureTransport>(sent),
                                                     gw::FollowOwnerOptions{});
    seed(fc, now, "position");
    std::vector<std::int64_t> received;
    std::int64_t boot_ms = 1000;  // FC 시각이 늘지 않는 ATTITUDE는 ingest가 버린다
    const auto feed = [&](double at, double roll = 0.0) {
        now = at;
        gw::InboundMessage message;
        message.kind = "ATTITUDE";
        message.system = 1;
        message.component = 1;
        message.data = {
            {"time_boot_ms", ++boot_ms}, {"roll", roll},      {"pitch", 0.0},   {"yaw", at - 299.0},
            {"rollspeed", 0.0},          {"pitchspeed", 0.0}, {"yawspeed", 0.0}};
        link.ingest(message);
        received.push_back(static_cast<std::int64_t>(at * 1e9));
    };
    const auto topic_of = [](const gw::Parts& parts) {
        return std::string(parts.at(0).begin(), parts.at(0).end());
    };
    struct Tick {
        int states = 0;
        std::vector<std::int64_t> stamps;
        std::vector<double> yaws;
    };
    const auto step_at = [&](double at) {
        now = at;
        sent.clear();
        link.step();
        Tick tick;
        for (const auto& parts : sent) {
            const std::string topic = topic_of(parts);
            if (topic == "fc/state") {
                ++tick.states;
            } else if (topic == "fc/attitude") {
                const auto value = gw::Json::from_msgpack(parts.at(1));
                tick.stamps.push_back(value.at("received_monotonic_ns").get<std::int64_t>());
                tick.yaws.push_back(value.at("yaw_rad").get<double>());
            }
        }
        return tick;
    };
    const auto tail = [&](std::size_t count) {
        const auto begin = std::prev(received.end(), static_cast<std::ptrdiff_t>(count));
        return std::vector<std::int64_t>(begin, received.end());
    };

    // 첫 송신: 이력 3개가 있어도 최신 표본 하나만 보낸다
    feed(299.97);
    feed(299.98);
    feed(299.99);
    require(fc.pose_history.at("ATTITUDE").size() == 3, "ingest fills ATTITUDE history");
    Tick tick = step_at(300.0);
    require(tick.states == 1 && tick.stamps == tail(1), "first tick does not dump history");

    // 10 ms 루프(즉시 송신 경로)에 표본 2개: 둘 다 수신 순서대로, 중복 없이
    feed(300.004);
    feed(300.008);
    tick = step_at(300.010);
    require(tick.states == 0, "10 ms tick is the immediate attitude path");
    require(tick.stamps == tail(2), "both samples of one loop are published in order");
    require(tick.yaws.size() == 2 && tick.yaws.at(0) < tick.yaws.at(1),
            "each published sample carries its own data");
    tick = step_at(300.020);
    require(tick.states == 0 && tick.stamps.empty(), "no new sample, no attitude");

    // 30 ms 주기 송신 경로: 아직 안 보낸 표본을 먼저 차례로 보낸다. 묶음의 마지막과 같은 최신
    // 표본은 같은 루프에서 다시 보내지 않는다 (2026-11-25 리뷰)
    feed(300.024);
    feed(300.028);
    tick = step_at(300.035);
    require(tick.states == 1, "30 ms tick publishes fc/state");
    require(tick.stamps == tail(2), "30 ms publish sends pending samples once, in order");

    // 한 루프에 20개가 쌓이면 가장 새 16개만 순서대로 보낸다
    for (int i = 0; i < 20; ++i) {
        feed(300.0355 + (i * .0004));
    }
    tick = step_at(300.045);
    require(tick.states == 0 && tick.stamps == tail(16), "burst is capped at the newest 16");
    tick = step_at(300.055);
    require(tick.states == 0 && tick.stamps.empty(), "burst is not repeated");

    // 새 표본 없는 30 ms 주기는 최신 표본 하나만 (기존 동작)
    tick = step_at(300.066);
    require(tick.states == 1 && tick.stamps == tail(1),
            "periodic publish without new samples repeats only the latest");

    // 2026-11-25 리뷰: 밀린 표본이 정확히 16개이면 모두 보낸다
    for (int i = 0; i < 16; ++i) {
        feed(300.0665 + (i * .0004));
    }
    tick = step_at(300.075);
    require(tick.states == 0 && tick.stamps == tail(16), "exactly 16 pending samples are sent");

    // 유한하지 않은 최신 표본은 이력에 없다. 이력의 앞 표본을 보낸 뒤 최신 표본으로 보낸다
    feed(300.078);
    feed(300.080, std::nan(""));
    require(fc.pose_history.at("ATTITUDE").back().at("received_monotonic_ns") == tail(2).at(0),
            "non-finite sample stays out of history");
    tick = step_at(300.085);
    require(tick.states == 0 && tick.stamps == tail(2),
            "immediate path sends history then the non-finite latest");
    feed(300.088);
    feed(300.090, std::nan(""));
    tick = step_at(300.100);
    require(tick.states == 1 && tick.stamps == tail(2),
            "30 ms path sends history then the non-finite latest");

    // connect, FC 재부팅처럼 이력과 표본이 비고 pose_epoch가 바뀐 뒤에도 새 표본을 한 번씩
    // 보낸다. 실제 재부팅 경로는 MAVLink 연결이 있어야 해서, 그 경로가 바꾸는 상태만 흉내 낸다
    fc.samples.clear();
    for (auto& entry : fc.pose_history) {
        entry.second.clear();
    }
    fc.pose_epoch += 1;
    feed(300.104);
    require(fc.pose_history.at("ATTITUDE").size() == 1, "history restarts after the epoch change");
    tick = step_at(300.110);
    require(tick.stamps == tail(1), "first sample after reboot is sent once");
    feed(300.113);
    feed(300.116);
    tick = step_at(300.120);
    require(tick.stamps == tail(2), "samples after reboot are sent in order");
}
}  // namespace
int main() {
    indoor_offboard_reclaim_test();
    try {
        authority_test();
        neutral_test();
        reclaim_test(false);
        reclaim_test(true);
        calibration_retry_test();
        attitude_publish_test();
        attitude_burst_test();
        std::cout << "operator exclusion, RC neutral, HOLD/STOP correlation and reclaim "
                     "cancellation passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
