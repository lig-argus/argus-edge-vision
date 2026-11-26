// 2026-11-18 실내 데모 위치 고정 시험: 데모 추종은 LOCAL_NED 위치 유지 + yaw rate(frame 1, mask
// 1528)로 보내고, 비데모는 기존 BODY_NED 속도(frame 8, mask 1479)를 그대로 보낸다.
// FC 없는 FlightLink와 가짜 MAVLink 링크만 쓴다 (소켓, FC, 카메라는 열지 않음).
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "argus/gateway/follow/include/follow_owner.hpp"
#include "argus/gateway/vehicle/include/flight_link.hpp"

namespace gw = argus::gateway;

namespace {

int failures = 0;
int checks = 0;

void check(bool value, const std::string& message) {
    ++checks;
    if (!value) {
        ++failures;
        std::cerr << "FAIL " << message << '\n';
    }
}

bool near(double a, double b, double tolerance = 1e-4) { return std::fabs(a - b) <= tolerance; }

constexpr const char* kUid = "000600000000353733323333511700280027";

class FakeLink final : public gw::ILink {
public:
    explicit FakeLink(gw::MavlinkParser& sent) : sent_(sent) {}
    gw::Bytes read_available() override { return {}; }
    void write(const gw::Bytes& data) override { sent_.feed(data); }
    void close() override {}

private:
    gw::MavlinkParser& sent_;
};

class NullTransport final : public gw::IFollowTransport {
public:
    void publish(const gw::Parts& /*parts*/) override {}
    std::optional<gw::Parts> rpc_receive() override { return std::nullopt; }
    void rpc_send(const gw::Parts& /*parts*/) override {}
    std::optional<gw::Parts> command_receive() override { return std::nullopt; }
    bool has_tracks() const override { return false; }
    std::optional<gw::Parts> track_receive() override { return std::nullopt; }
    void close() override {}
};

struct Rig {
    double now = 100.0;
    int ids = 0;
    std::optional<bool> demo;
    gw::MavlinkParser sent;
    std::unique_ptr<gw::FlightLink> link;
    gw::FollowBOwner* owner = nullptr;
    std::string token;
    std::int64_t seq = 0;
    double x = 2.0;
    double y = 3.0;
    double z = -1.7;

    Rig() {
        gw::FlightLinkOptions options;
        options.expected_uid = kUid;
        options.event_metadata = "/nonexistent/fc-events.json";
        options.clock = [this] { return now; };
        options.new_id = [this] { return "id" + std::to_string(++ids); };
        link = std::make_unique<gw::FlightLink>(std::move(options));
        auto& fc = link->state();
        fc.link = std::make_unique<FakeLink>(sent);
        fc.mav = std::make_unique<gw::MavlinkSender>(245, 191);
        fc.uid = link->expected_uid();
        fc.config_requested = true;
        // 실내 프로필 (GatewayStage3Test와 같은 값)
        for (const auto& [key, value] : gw::indoor_profile()) {
            fc.params.insert_or_assign(key, value);
        }
        fc.params.insert_or_assign("EKF2_OF_QMIN", 1);
        fc.params.insert_or_assign("COM_POS_FS_EPH", 5);
        fc.params.insert_or_assign("SENS_FLOW_MAXHGT", 2);
        auto created = std::make_unique<gw::FollowBOwner>(*link, std::make_unique<NullTransport>(),
                                                          gw::FollowOwnerOptions{});
        owner = created.get();
        fc.follow_b = std::move(created);
        link->set_demo_follow_source([this] { return demo; });
    }

    std::int64_t now_ns() const { return static_cast<std::int64_t>(now * 1'000'000'000); }

    void seed(const std::string& mode) {
        auto& fc = link->state();
        const auto set = [&](const char* name, gw::Json data) {
            fc.samples.insert_or_assign(name, gw::Sample{std::move(data), now});
        };
        const auto numbers = gw::mode_numbers(mode).value_or(std::pair<int, int>{0, 0});
        set("HEARTBEAT",
            {{"base_mode", 128}, {"custom_mode", (numbers.first << 16) | (numbers.second << 24)}});
        set("EXTENDED_SYS_STATE", {{"landed_state", 2}});
        set("ESTIMATOR_STATUS", {{"flags", 79}, {"pos_horiz_accuracy", .1}});
        set("LOCAL_POSITION_NED", {{"x", x}, {"y", y}, {"z", z}, {"vx", 0}, {"vy", 0}, {"vz", 0}});
        set("ATTITUDE", {{"roll", 0}, {"pitch", 0}, {"yaw", .2}});
        // 지면 기준 높이 1.70 m (레이저) -> 지면 local z = z + 1.70
        set("DISTANCE_SENSOR",
            {{"current_distance", 170}, {"min_distance", 8}, {"max_distance", 400}});
        set("OPTICAL_FLOW_RAD", {{"quality", 50}});
    }

    gw::Wire base(const char* schema) const {
        gw::Wire d = gw::Wire::object();
        gw::wire_set(d, "schema", schema);
        gw::wire_set(d, "source", "argus-follow-controller");
        gw::wire_set(d, "clock_domain", "host_monotonic");
        gw::wire_set(d, "uid", kUid);
        gw::wire_set(d, "onboard_session", owner->session());
        gw::wire_set(d, "fc_epoch", link->state().pose_epoch);
        gw::wire_set(d, "client_session", "controller");
        gw::wire_set(d, "revision", 1);
        gw::wire_set(d, "command_epoch", 1);
        gw::wire_set(d, "generation", 1);
        gw::wire_set(d, "track_id", 7);
        gw::wire_set(d, "source_sample_monotonic_ns", now_ns());
        gw::wire_set(d, "issued_monotonic_ns", now_ns());
        gw::wire_set(d, "valid_until_monotonic_ns", now_ns() + 200'000'000);
        gw::wire_set(d, "bench", false);
        return d;
    }

    bool acquire() {
        gw::Wire d = base("argus.follow.rpc.v1");
        gw::wire_set(d, "request_id", "acquire" + std::to_string(++ids));
        gw::wire_set(d, "seq", ++seq);
        gw::wire_set(d, "action", "acquire");
        const gw::Wire reply = owner->request(d);
        if (reply.at("status") != "accepted") {
            std::cerr << "acquire: " << reply.dump() << '\n';
            return false;
        }
        token = reply.at("owner_token").get<std::string>();
        return true;
    }

    bool setpoint(double vx, double vy, double vz, double yaw_rate) {
        gw::Wire d = base("argus.control.setpoint.v1");
        gw::wire_set(d, "request_id", "sp" + std::to_string(++ids));
        gw::wire_set(d, "seq", ++seq);
        gw::wire_set(d, "owner_token", token);
        gw::wire_set(d, "action", "velocity");
        gw::wire_set(d, "coordinate_frame", "body_frd");
        gw::wire_set(d, "vx_m_s", vx);
        gw::wire_set(d, "vy_m_s", vy);
        gw::wire_set(d, "vz_m_s", vz);
        gw::wire_set(d, "yaw_rate_rad_s", yaw_rate);
        return owner->receive_setpoint(d);
    }

    // 시간을 dt만큼 옮기고 표본을 새로 넣은 뒤 명령 하나를 보내고 송신부를 한 번 돌린다.
    std::vector<gw::Json> step(double dt, double vx, double vy, double vz, double yaw_rate,
                               const std::string& mode = "offboard") {
        now += dt;
        seed(mode);
        drain();
        const bool accepted = setpoint(vx, vy, vz, yaw_rate);
        if (!accepted) {
            std::cerr << "setpoint rejected: " << owner->last_reason() << '\n';
        }
        owner->tick();
        return drain();
    }

    std::vector<gw::Json> drain() {
        std::vector<gw::Json> targets;
        while (const auto packet = sent.next()) {
            if (packet->kind == "SET_POSITION_TARGET_LOCAL_NED") {
                targets.push_back(packet->data);
            }
        }
        return targets;
    }

    bool has_event(const std::string& prefix) const {
        for (const auto& event : link->state().events) {
            const gw::Json text =
                event.is_object() && event.contains("text") ? event.at("text") : gw::Json(event);
            if (text.is_string() && text.get<std::string>().rfind(prefix, 0) == 0) {
                return true;
            }
        }
        return false;
    }
};

// 위치 유지 + yaw rate 송신인가 (x, y, z, yaw_rate 확인)
bool position_hold(const gw::Json& p, double x, double y, double z, double yaw_rate) {
    return p.at("coordinate_frame") == 1 && p.at("type_mask") == 1528 &&
           near(p.at("x").get<double>(), x) && near(p.at("y").get<double>(), y) &&
           near(p.at("z").get<double>(), z) && near(p.at("yaw_rate").get<double>(), yaw_rate) &&
           p.at("vx") == 0 && p.at("vy") == 0 && p.at("vz") == 0;
}

void non_demo_keeps_body_velocity() {
    for (const std::optional<bool> demo : {std::optional<bool>(), std::optional<bool>(false)}) {
        Rig rig;
        rig.demo = demo;
        rig.seed("position");
        check(rig.acquire(), "non-demo acquire");
        const auto zero = rig.step(.1, 0, 0, 0, 0, "position");
        check(zero.size() == 1 && zero.at(0).at("coordinate_frame") == 8 &&
                  zero.at(0).at("type_mask") == 1479,
              "non-demo zero setpoint stays BODY_NED velocity mask 1479");
        const auto moving = rig.step(.1, .5, -.2, .1, .3);
        check(moving.size() == 1 && moving.at(0).at("coordinate_frame") == 8 &&
                  moving.at(0).at("type_mask") == 1479 &&
                  near(moving.at(0).at("vx").get<double>(), .5) &&
                  near(moving.at(0).at("vy").get<double>(), -.2) &&
                  near(moving.at(0).at("vz").get<double>(), .1) &&
                  near(moving.at(0).at("yaw_rate").get<double>(), .3) &&
                  moving.at(0).at("x") == 0 && moving.at(0).at("z") == 0,
              "non-demo translation stays BODY_NED velocity");
        check(rig.owner->token_active(), "non-demo translation accepted");
        check(rig.owner->demo_hold_status().at("demo_position_hold") == false,
              "non-demo has no position hold");
    }
}

void demo_position_hold_and_z_band() {
    Rig rig;
    rig.demo = true;
    rig.seed("position");
    check(rig.acquire(), "demo acquire");
    // 영명령(틈, 제어기 멈춤)도 위치 유지 + yaw rate 0으로 나간다
    auto sent = rig.step(.1, 0, 0, 0, 0, "position");
    check(sent.size() == 1 && position_hold(sent.at(0), 2.0, 3.0, -1.7, 0),
          "demo zero setpoint is LOCAL_NED position hold yaw_rate 0");
    check(rig.has_event("데모 위치 고정 x=2.00 y=3.00 z=-1.70"), "anchor capture event");
    const gw::Wire status = rig.owner->demo_hold_status();
    check(status.at("demo_position_hold") == true && near(status.at("anchor_x").get<double>(), 2) &&
              near(status.at("anchor_y").get<double>(), 3),
          "anchor exposed in demo_hold_status");
    // 기체가 기준점 근처에서 조금 움직여도 기준점은 그대로다
    rig.x = 2.3;
    rig.y = 2.8;
    sent = rig.step(.1, 0, 0, .5, .4);
    check(sent.size() == 1 && position_hold(sent.at(0), 2.0, 3.0, -1.65, .4),
          "anchor x,y fixed, z integrates vz*dt (0.5*0.1)");
    // 송신 간격이 길어도 dt는 0.2 s로 자른다
    sent = rig.step(.5, 0, 0, -1.0, -.2);
    check(sent.size() == 1 && position_hold(sent.at(0), 2.0, 3.0, -1.85, -.2),
          "dt clamped to 0.2 s");
    // 위로 계속: 지면 위 2.0 m(z=-2.0)에서 멈춘다 (지면 local z = -1.7 + 1.7 = 0)
    for (int i = 0; i < 4; ++i) {
        sent = rig.step(.1, 0, 0, -1.0, 0);
    }
    check(sent.size() == 1 && position_hold(sent.at(0), 2.0, 3.0, -2.0, 0),
          "z target clamped to demo max height 2.0 m");
    // 아래로 계속: 지면 위 1.0 m(z=-1.0)에서 멈춘다
    for (int i = 0; i < 6; ++i) {
        sent = rig.step(.1, 0, 0, 2.0, 0);
    }
    check(sent.size() == 1 && position_hold(sent.at(0), 2.0, 3.0, -1.0, 0),
          "z target clamped to demo min height 1.0 m");
    check(rig.owner->token_active(), "demo ownership kept");
    // 상태를 잠깐 모르게 돼도(비어 있음) 데모 송신을 유지한다
    rig.demo.reset();
    sent = rig.step(.1, 0, 0, 0, .1);
    check(sent.size() == 1 && position_hold(sent.at(0), 2.0, 3.0, -1.0, .1),
          "unknown profile keeps latched demo hold");
}

void demo_rejects_translation() {
    for (const auto& [vx, vy] : {std::pair<double, double>{.3, 0}, {0, -.3}}) {
        Rig rig;
        rig.demo = true;
        rig.seed("position");
        check(rig.acquire(), "demo acquire for translation");
        rig.step(.1, 0, 0, 0, 0, "position");
        const auto sent = rig.step(.1, vx, vy, 0, .2);
        check(!rig.owner->token_active() && rig.owner->last_reason() == "demo_translation_rejected",
              "demo nonzero vx/vy rejected through safety_stop");
        bool translated = false;
        for (const auto& p : sent) {
            translated = translated || p.at("vx") != 0 || p.at("vy") != 0;
        }
        check(!translated, "rejected translation never sent");
        check(!rig.link->state().jobs.empty() &&
                  rig.link->state().jobs.front()->at("action") == "land",
              "existing safety_stop path queued (no hover option -> land)");
        check(rig.owner->demo_hold_status().at("demo_position_hold") == false,
              "anchor reset after rejection");
    }
}

void demo_anchor_reset_on_epoch() {
    Rig rig;
    rig.demo = true;
    rig.seed("position");
    check(rig.acquire(), "demo acquire for epoch");
    rig.step(.1, 0, 0, 0, 0, "position");
    check(rig.owner->demo_hold_status().at("demo_position_hold") == true, "anchor before epoch");
    // EKF 재설정: local 좌표가 바뀐다
    rig.link->state().pose_epoch += 1;
    rig.x = 5.0;
    rig.y = -1.0;
    rig.now += .05;
    rig.seed("offboard");
    rig.owner->tick();
    check(!rig.owner->token_active() && rig.owner->last_reason() == "fc_epoch_changed",
          "epoch change invalidates ownership");
    check(rig.owner->demo_hold_status().at("demo_position_hold") == false &&
              rig.owner->demo_hold_status().at("anchor_x").is_null(),
          "anchor reset on epoch change");
    check(rig.has_event("데모 위치 고정 해제 (fc_epoch_changed)"), "anchor reset event");
    // 새 소유권은 새 epoch의 현재 위치로 기준점을 다시 잡는다
    rig.drain();
    rig.seed("position");
    check(rig.acquire(), "demo re-acquire after epoch");
    const auto sent = rig.step(.1, 0, 0, 0, 0, "position");
    check(sent.size() == 1 && position_hold(sent.at(0), 5.0, -1.0, -1.7, 0),
          "new anchor from new epoch position");
    check(rig.owner->demo_hold_status().at("anchor_epoch") == rig.link->state().pose_epoch,
          "anchor epoch recorded");
}

void demo_anchor_drift_rejected() {
    Rig rig;
    rig.demo = true;
    rig.seed("position");
    check(rig.acquire(), "demo acquire for drift");
    rig.step(.1, 0, 0, 0, 0, "position");
    rig.x = 2.0 + .7;
    rig.y = 3.0 + .7;  // 수평 0.99 m: 아직 허용
    auto sent = rig.step(.1, 0, 0, 0, .1);
    check(sent.size() == 1 && position_hold(sent.at(0), 2.0, 3.0, -1.7, .1),
          "drift under 1.0 m keeps anchor");
    rig.x = 2.0 + .8;  // 수평 1.06 m
    sent = rig.step(.1, 0, 0, 0, .1);
    check(!rig.owner->token_active() && rig.owner->last_reason() == "demo_anchor_drift",
          "drift over 1.0 m rejected through safety_stop");
    bool reanchored = false;
    for (const auto& p : sent) {
        reanchored = reanchored || p.at("coordinate_frame") == 1;
    }
    check(!reanchored, "no silent re-anchor after drift");
}

void demo_profile_change_rejected() {
    Rig rig;
    rig.demo = true;
    rig.seed("position");
    check(rig.acquire(), "demo acquire for profile change");
    rig.step(.1, 0, 0, 0, 0, "position");
    rig.demo = false;
    rig.step(.1, 0, 0, 0, 0, "position");
    check(!rig.owner->token_active() && rig.owner->last_reason() == "demo_profile_changed",
          "confirmed non-demo during demo ownership rejected");
    check(rig.has_event("데모 위치 고정 해제 (demo_profile_inactive)"), "profile reset event");
}

// 기존 Offboard 고도 유지에서 바로 인계: 기준점은 인계 직전 고도 유지 목표다
void demo_anchor_from_handoff_goal() {
    Rig rig;
    rig.demo = true;
    auto& fc = rig.link->state();
    rig.seed("offboard");
    fc.owner = "main:test";
    fc.gcs_client = "main:test";
    fc.gcs_at = rig.now;
    fc.external = fc.owned_flight = fc.owned_mode_confirmed = fc.holding = true;
    fc.local_target = std::make_shared<gw::Json>(gw::Json{{"x", 1.9},
                                                          {"y", 3.1},
                                                          {"start_z", -1.6},
                                                          {"target_z", -1.6},
                                                          {"distance", 0.0},
                                                          {"height", 1.6},
                                                          {"yaw", .2},
                                                          {"climb_at", rig.now - 5}});
    bool prepared = true;
    try {
        rig.owner->prepare_handoff();
    } catch (const std::exception& error) {
        prepared = false;
        std::cerr << "prepare_handoff: " << error.what() << '\n';
    }
    check(prepared, "prepare_handoff accepted");
    check(rig.acquire(), "direct acquire from offboard hold");
    fc.gcs_at = rig.now + .1;
    const auto sent = rig.step(.1, 0, 0, 0, .25);
    check(sent.size() == 1 && position_hold(sent.at(0), 1.9, 3.1, -1.6, .25),
          "anchor from handoff goal x,y and goal z");
    check(!fc.external && !fc.local_target && rig.owner->token_active(),
          "handoff confirmed after demo send");
}

// 인계 직전 고도 유지 상태를 만든다 (demo_anchor_from_handoff_goal과 같은 값)
bool prepare_offboard_hold(Rig& rig) {
    auto& fc = rig.link->state();
    rig.seed("offboard");
    fc.owner = "main:test";
    fc.gcs_client = "main:test";
    fc.gcs_at = rig.now;
    fc.external = fc.owned_flight = fc.owned_mode_confirmed = fc.holding = true;
    fc.local_target = std::make_shared<gw::Json>(gw::Json{{"x", 1.9},
                                                          {"y", 3.1},
                                                          {"start_z", -1.6},
                                                          {"target_z", -1.6},
                                                          {"distance", 0.0},
                                                          {"height", 1.6},
                                                          {"yaw", .2},
                                                          {"climb_at", rig.now - 5}});
    try {
        rig.owner->prepare_handoff();
    } catch (const std::exception& error) {
        std::cerr << "prepare_handoff: " << error.what() << '\n';
        return false;
    }
    return true;
}

// 인계 대기 중 프로필을 모르면 속도 송신으로 인계를 확정하지 않고 미룬다.
void demo_unknown_during_handoff_deferred() {
    Rig rig;
    rig.demo.reset();
    auto& fc = rig.link->state();
    check(prepare_offboard_hold(rig), "prepare_handoff for unknown profile");
    check(rig.acquire(), "acquire with unknown profile");
    fc.gcs_at = rig.now + .1;
    auto sent = rig.step(.1, 0, 0, 0, .25);
    check(sent.empty(), "unknown profile during handoff sends nothing");
    check(fc.external && fc.local_target && rig.owner->token_active(),
          "handoff not confirmed while profile unknown");
    // 판정이 들어오면 인계 직전 목표를 기준점으로 잡고 인계를 확정한다
    rig.demo = true;
    fc.gcs_at = rig.now + .1;
    sent = rig.step(.05, 0, 0, 0, .25);
    check(sent.size() == 1 && position_hold(sent.at(0), 1.9, 3.1, -1.6, .25),
          "deferred handoff anchors at hold goal once demo is known");
    check(!fc.external && !fc.local_target && rig.owner->token_active(),
          "handoff confirmed by the first demo send");
}

// 비데모로 시작한 소유권 안에서 demo로 바뀌어도 위치 송신으로 바꾸지 않고 거부한다.
void non_demo_to_demo_rejected() {
    for (const std::optional<bool> first : {std::optional<bool>(false), std::optional<bool>()}) {
        Rig rig;
        rig.demo = first;
        rig.seed("position");
        check(rig.acquire(), "non-demo acquire for change");
        const auto zero = rig.step(.1, 0, 0, 0, 0, "position");
        check(zero.size() == 1 && zero.at(0).at("coordinate_frame") == 8,
              "first send is BODY_NED velocity");
        rig.demo = true;
        const auto sent = rig.step(.1, 0, 0, 0, .1);
        bool position = false;
        for (const auto& p : sent) {
            position = position || p.at("coordinate_frame") == 1;
        }
        check(!position && !rig.owner->token_active() &&
                  rig.owner->last_reason() == "demo_profile_changed",
              "non-demo to demo inside one ownership rejected");
    }
}

// 지면 높이를 모르면 z 목표를 멈추고 한 번 남긴다.
void demo_height_unknown_freezes_z() {
    Rig rig;
    rig.demo = true;
    rig.seed("position");
    check(rig.acquire(), "demo acquire for height unknown");
    rig.step(.1, 0, 0, 0, 0, "position");
    rig.now += .1;
    rig.seed("offboard");
    rig.link->state().samples.erase("DISTANCE_SENSOR");
    rig.drain();
    check(!rig.link->height().first, "height unknown in rig");
    rig.setpoint(0, 0, .5, 0);
    rig.owner->tick();
    const auto sent = rig.drain();
    check(sent.size() == 1 && position_hold(sent.at(0), 2.0, 3.0, -1.7, 0),
          "unknown height keeps z target");
    check(rig.has_event("데모 고도 유지 멈춤"), "height unknown event");
}

}  // namespace

int main() {
    try {
        non_demo_keeps_body_velocity();
        demo_position_hold_and_z_band();
        demo_rejects_translation();
        demo_anchor_reset_on_epoch();
        demo_anchor_drift_rejected();
        demo_profile_change_rejected();
        demo_anchor_from_handoff_goal();
        demo_unknown_during_handoff_deferred();
        non_demo_to_demo_rejected();
        demo_height_unknown_freezes_z();
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures != 0) {
        std::cerr << failures << "/" << checks << " 실패\n";
        return 1;
    }
    std::cout << "GatewayDemoAnchorTest 통과 (" << checks << " 검사)\n";
    return 0;
}
