// 게이트웨이 2~6단계 비교 도구. 표준 입력의 JSON 줄(op)로 FlightLink를 움직이고 결과를 JSON 줄로
// 낸다. 링크는 가짜다: 입력 바이트는 op로 넣고, FC로 나간 프레임은 op마다 16진수로 낸다.
// tests/gateway/differential/stage2~5b_diff.py가 같은 op를 Python 원본에 넣고 맞춰 본다.
// 5b: 메인 GUI 연결부(MainGcsBridge)의 두 스레드를 Gate로 한 칸씩 진행한다.
// 6: HTTP API(GatewayApi)에 원본 HTTP 서버와 같은 요청 머리·본문을 넣는다 (stage6_diff.py).
// 실제 장치·FC·네트워크를 열지 않는다.
#include <unistd.h>

#include <array>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "argus/gateway/bridge/include/follow_preferences.hpp"
#include "argus/gateway/bridge/include/main_gcs_bridge.hpp"
#include "argus/gateway/bridge/include/profile_coordinator.hpp"
#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/common/include/python_compat.hpp"
#include "argus/gateway/follow/include/follow_owner.hpp"
#include "argus/gateway/http/include/gateway_api.hpp"
#include "argus/gateway/mavlink/include/mavlink_codec.hpp"
#include "argus/gateway/ownership/include/operator_authority.hpp"
#include "argus/gateway/service/include/maintenance.hpp"
#include "argus/gateway/settings/include/deployment_settings.hpp"
#include "argus/gateway/vehicle/include/flight_link.hpp"
#include "argus/gateway/vehicle/include/rc_reclaim.hpp"

namespace gw = argus::gateway;
using nlohmann::json;

namespace {

gw::Bytes hex_bytes(const std::string& hex) {
    gw::Bytes out;
    for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    }
    return out;
}

struct Channel {
    gw::Bytes inbound;
    bool open{true};
};

struct Recorder {
    std::vector<std::string> frames;
    std::shared_ptr<Channel> current;
    json opened = json::array();
};

class FakeLink final : public gw::ILink {
public:
    FakeLink(std::shared_ptr<Channel> channel, Recorder& recorder)
        : channel_(std::move(channel)), recorder_(recorder) {}
    FakeLink(const FakeLink&) = delete;
    FakeLink& operator=(const FakeLink&) = delete;
    FakeLink(FakeLink&&) = delete;
    FakeLink& operator=(FakeLink&&) = delete;
    ~FakeLink() override = default;

    gw::Bytes read_available() override {
        gw::Bytes out;
        out.swap(channel_->inbound);
        return out;
    }
    void write(const gw::Bytes& bytes) override {
        recorder_.frames.push_back(gw::hex_lower(bytes));
    }
    void close() override { channel_->open = false; }

private:
    std::shared_ptr<Channel> channel_;
    Recorder& recorder_;
};

// 호출 기록 (추종 송신부·추종기 RPC). op마다 비운다
json& calls() {
    static json log = json::array();
    return log;
}

class FakeManualYaw final : public gw::IManualYaw {
public:
    bool active() const override { return is_active; }
    json status() const override { return {{"supported", true}, {"active", is_active}}; }
    std::optional<double> position_rate() override { return rate; }
    json request(const json& /*body*/) override { return json::object(); }
    void cancel(const std::string& reason) override {
        calls().push_back({"manual_cancel", reason});
    }
    bool is_active{false};
    std::optional<double> rate;
};

// 원본 FollowBOwner를 대신하는 기록용 송신부
class FakeFollow final : public gw::IFollowSupervisor {
public:
    void ingest(const std::string& /*kind*/, const json& /*data*/) override { ++ingests; }
    gw::IManualYaw& manual_yaw() override { return yaw; }
    const gw::IManualYaw& manual_yaw() const override { return yaw; }
    bool token_active() const override { return token; }
    bool bench_active() const override { return bench; }
    void clear_bench() override {
        bench = false;
        calls().push_back({"bench", nullptr});
    }
    void invalidate(const std::string& reason) override {
        calls().push_back({"invalidate", reason});
    }
    bool offboard_hover_enabled() const override { return hover_enabled; }
    bool establish_hover(const std::string& reason, bool connection_lost) override {
        calls().push_back({"establish_hover", reason, connection_lost});
        return hover_result;
    }
    void tick() override { ++ticks; }
    bool rc_override() const override { return rc_override_value; }
    void set_rc_override(bool value) override {
        rc_override_value = value;
        calls().push_back({"rc_override", value});
    }
    void prepare_handoff() override { calls().push_back(json::array({"prepare_handoff"})); }
    bool legacy_mode_handoff() const override { return legacy; }
    FakeManualYaw yaw;
    bool token{false};
    bool bench{false};
    bool hover_enabled{false};
    bool hover_result{false};
    bool rc_override_value{false};
    bool legacy{false};
    int ingests{0};
    int ticks{0};
};

// 복귀 절차 스레드를 대기 지점마다 한 칸씩 진행시킨다 (Python 쪽 시험과 같은 순서)
struct Gate {
    std::mutex mutex;
    std::condition_variable changed;
    bool active{false};
    bool waiting{false};
    bool go{false};
    bool shutdown{false};
    long generation{0};
    double requested{0};

    bool wait(double seconds) {
        std::unique_lock<std::mutex> lock(mutex);
        if (shutdown) {
            return true;
        }
        waiting = true;
        requested = seconds;
        ++generation;
        changed.notify_all();
        changed.wait(lock, [this] { return go || shutdown; });
        go = false;
        waiting = false;
        return shutdown;
    }
    void finished() {
        const std::scoped_lock<std::mutex> lock(mutex);
        active = false;
        changed.notify_all();
    }
    void settle() {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [this] { return !active || waiting; });
    }
    bool advance() {
        std::unique_lock<std::mutex> lock(mutex);
        if (!active || !waiting) {
            return false;
        }
        const long before = generation;
        go = true;
        changed.notify_all();
        changed.wait(lock, [this, before] { return !active || (waiting && generation != before); });
        return true;
    }
    json state() {
        const std::scoped_lock<std::mutex> lock(mutex);
        return active && waiting ? json(requested) : json();
    }
    void close() {
        const std::scoped_lock<std::mutex> lock(mutex);
        shutdown = true;
        changed.notify_all();
    }
};

// 원본 FollowBOwner의 ZMQ 소켓 대신: 들어올 메시지는 op로 넣고, 나간 메시지는 op마다 낸다
struct OwnerWire {
    std::deque<gw::Parts> rpc_in;
    std::deque<gw::Parts> command_in;
    std::deque<gw::Parts> track_in;
    json published = json::array();
    json replies = json::array();
    bool tracks{false};
};

json parts_hex(const gw::Parts& parts) {
    json out = json::array();
    for (const auto& part : parts) {
        out.push_back(gw::hex_lower(part));
    }
    return out;
}

class FakeOwnerTransport final : public gw::IFollowTransport {
public:
    explicit FakeOwnerTransport(OwnerWire& wire) : wire_(wire) {}
    void publish(const gw::Parts& parts) override { wire_.published.push_back(parts_hex(parts)); }
    std::optional<gw::Parts> rpc_receive() override { return pop(wire_.rpc_in); }
    void rpc_send(const gw::Parts& parts) override { wire_.replies.push_back(parts_hex(parts)); }
    std::optional<gw::Parts> command_receive() override { return pop(wire_.command_in); }
    bool has_tracks() const override { return wire_.tracks; }
    std::optional<gw::Parts> track_receive() override { return pop(wire_.track_in); }
    void close() override {}

private:
    static std::optional<gw::Parts> pop(std::deque<gw::Parts>& queue) {
        if (queue.empty()) {
            return std::nullopt;
        }
        gw::Parts out = std::move(queue.front());
        queue.pop_front();
        return out;
    }
    OwnerWire& wire_;
};

gw::Parts hex_parts(const json& value) {
    gw::Parts out;
    for (const auto& part : value) {
        out.push_back(hex_bytes(part.get<std::string>()));
    }
    return out;
}

json error_json(const std::exception& error) {
    const bool value_error = dynamic_cast<const gw::ValueError*>(&error) != nullptr;
    return {{"error", error.what()}, {"type", value_error ? "ValueError" : "Other"}};
}

void emit(const json& value) { std::cout << gw::python_json_dumps_sorted(value) << '\n'; }
// 형식을 보존한 JSON 표현: 실수 {"$f": repr}, bytes {"$b": hex}, 사전 {"$m": [[키, 값], ...]}
// tagged/untagged는 중첩 값을 재귀로 바꾼다 (깊이는 입력 구조만큼)
json tagged(const gw::Wire& value) {  // NOLINT(misc-no-recursion)
    if (value.is_number_float()) {
        return {{"$f", gw::python_float_repr(value.get<double>())}};
    }
    if (value.is_binary()) {
        return {
            {"$b", gw::hex_lower(gw::Bytes(value.get_binary().begin(), value.get_binary().end()))}};
    }
    if (value.is_array()) {
        json out = json::array();
        for (const auto& item : value) {
            out.push_back(tagged(item));
        }
        return out;
    }
    if (value.is_object()) {
        json pairs = json::array();
        for (const auto& item : value.items()) {
            pairs.push_back(json::array({item.key(), tagged(item.value())}));
        }
        return {{"$m", pairs}};
    }
    if (value.is_number_unsigned()) {
        return value.get<std::uint64_t>();
    }
    if (value.is_number_integer()) {
        return value.get<std::int64_t>();
    }
    if (value.is_boolean()) {
        return value.get<bool>();
    }
    if (value.is_string()) {
        return value.get<std::string>();
    }
    return nullptr;
}

gw::Wire untagged(const json& value) {  // NOLINT(misc-no-recursion)
    if (value.is_object() && value.contains("$f")) {
        const std::string text = value.at("$f");
        if (text == "nan") {
            return std::nan("");
        }
        if (text == "inf" || text == "-inf") {
            return text == "inf" ? HUGE_VAL : -HUGE_VAL;
        }
        return std::strtod(text.c_str(), nullptr);
    }
    if (value.is_object() && value.contains("$b")) {
        const gw::Bytes data = hex_bytes(value.at("$b"));
        return gw::Wire::binary(std::vector<std::uint8_t>(data.begin(), data.end()));
    }
    if (value.is_object() && value.contains("$m")) {
        gw::Wire out = gw::Wire::object();
        for (const auto& pair : value.at("$m")) {
            gw::wire_set(out, pair.at(0).get<std::string>(), untagged(pair.at(1)));
        }
        return out;
    }
    if (value.is_array()) {
        gw::Wire out = gw::Wire::array();
        for (const auto& item : value) {
            out.push_back(untagged(item));
        }
        return out;
    }
    return gw::Wire::parse(value.dump());
}

json tagged_float(double value) { return tagged(gw::Wire(value)); }

// 원본 MainGcsBridge의 external·router 소켓과 tracks SUB 대신
struct BridgeWire {
    std::deque<std::tuple<gw::Bytes, std::string, int>> external;
    std::deque<gw::Bytes> router;
    std::deque<gw::Parts> tracks;
    json gui_tx = json::array();
    json router_tx = json::array();
    bool router_fail{false};
    bool router_refused{false};  // 라우터가 꺼져 recv가 ECONNREFUSED를 내는 상태
};

class FakeBridgeSockets final : public gw::IBridgeSockets {
public:
    explicit FakeBridgeSockets(BridgeWire& wire) : wire_(wire) {}
    void wait_readable(double /*timeout_s*/) override {}
    std::optional<std::pair<gw::Bytes, gw::GcsAddress>> receive_external() override {
        if (wire_.external.empty()) {
            return std::nullopt;
        }
        auto [data, host, port] = wire_.external.front();
        wire_.external.pop_front();
        return std::make_pair(data, gw::GcsAddress{host, port});
    }
    std::optional<gw::Bytes> receive_router() override {
        if (wire_.router_refused) {
            throw gw::BridgeOsError("[Errno 111] Connection refused");
        }
        if (wire_.router.empty()) {
            return std::nullopt;
        }
        gw::Bytes data = wire_.router.front();
        wire_.router.pop_front();
        return data;
    }
    std::optional<std::vector<gw::Bytes>> receive_tracks() override {
        if (wire_.tracks.empty()) {
            return std::nullopt;
        }
        gw::Parts parts = wire_.tracks.front();
        wire_.tracks.pop_front();
        return parts;
    }
    void send_external(const gw::Bytes& data, const gw::GcsAddress& address) override {
        wire_.gui_tx.push_back({address.host, address.port, gw::hex_lower(data)});
    }
    void send_router(const gw::Bytes& data) override {
        if (wire_.router_fail) {
            throw gw::BridgeOsError("[Errno 111] Connection refused");
        }
        wire_.router_tx.push_back(gw::hex_lower(data));
    }
    void close() override {}

private:
    BridgeWire& wire_;
};

// 원본 쪽에서 기록한 추종기 응답을 차례로 돌려준다
struct FollowerReplay {
    std::deque<json> replies;
    json requests = json::array();
    json prepares = json::array();

    gw::Wire forward(const gw::Wire& body, int timeout_ms) {
        requests.push_back(json::array({tagged(body), timeout_ms}));
        if (replies.empty()) {
            throw gw::ValueError("응답 없음");
        }
        const json reply = replies.front();
        replies.pop_front();
        if (reply.is_object() && reply.contains("$error")) {
            throw gw::ValueError(reply.at("$error").get<std::string>());
        }
        return untagged(reply);
    }
};

// 브리지 두 스레드와 함께 쓰는 상태
struct BridgeRig {
    BridgeWire wire;
    FollowerReplay follower;
    std::unique_ptr<gw::FollowPreferences> preferences;
    std::unique_ptr<gw::FollowProfileCoordinator> profiles;
    std::unique_ptr<gw::MainGcsBridge> bridge;
    std::shared_ptr<Gate> io_gate = std::make_shared<Gate>();
    std::shared_ptr<Gate> worker_gate = std::make_shared<Gate>();
    std::thread io_thread;
    std::thread worker_thread;
    std::filesystem::path log_path;
    std::size_t log_seen{0};
    std::unique_ptr<gw::Maintenance> maintenance;
    std::unique_ptr<gw::GatewayApi> api;

    json log_lines() {
        json out = json::array();
        if (log_path.empty()) {
            return out;
        }
        std::ifstream file(log_path);
        std::string line;
        std::size_t index = 0;
        while (std::getline(file, line)) {
            if (index++ >= log_seen) {
                out.push_back(line);
            }
        }
        log_seen = index;
        return out;
    }

    static void start(const std::shared_ptr<Gate>& gate, std::thread& thread,
                      const std::function<void()>& body) {
        {
            const std::scoped_lock<std::mutex> lock(gate->mutex);
            gate->active = true;
        }
        thread = std::thread([gate, body] {
            body();
            gate->finished();
        });
        gate->settle();
    }

    void close() {
        if (bridge) {
            bridge->close();
        }
        io_gate->close();
        worker_gate->close();
        if (io_thread.joinable()) {
            io_thread.join();
        }
        if (worker_thread.joinable()) {
            worker_thread.join();
        }
    }
};

json bridge_state(const gw::Wire& raw) {
    json out = json::parse(raw.dump());
    for (auto& peer : out.at("peers")) {
        peer.at(3) = tagged_float(peer.at(3).get<double>());
    }
    for (auto& row : out.at("results")) {
        row.at(5) = tagged_float(row.at(5).get<double>());
    }
    out.at("last_tracks_at") = tagged_float(raw.at("last_tracks_at").get<double>());
    json& snapshot = out.at("snapshot");
    if (!snapshot.at("operator_status_age_s").is_null()) {
        snapshot.at("operator_status_age_s") =
            tagged_float(raw.at("snapshot").at("operator_status_age_s").get<double>());
    }
    return out;
}

int run_vehicle() {
    double now = 0.0;
    int next_id = 0;
    Recorder recorder;
    std::unique_ptr<gw::FlightLink> link;
    gw::OperatorAuthority authority;
    FakeFollow* follow = nullptr;
    auto gate = std::make_shared<Gate>();
    bool controller_present = false;
    std::deque<json> responses;
    std::unique_ptr<gw::CommandOwner> holder;
    OwnerWire wire;
    gw::FollowBOwner* owner = nullptr;
    std::string profile_path;
    BridgeRig rig;
    const std::function<std::string()> make_id = [&next_id] {
        const std::string digits = std::to_string(next_id++);
        const std::string padded =
            std::string(digits.size() < 4 ? 4 - digits.size() : 0, '0') + digits;
        return "id" + padded;
    };
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) {
            continue;
        }
        const json op = json::parse(line);
        const std::string name = op.at("op");
        recorder.frames.clear();
        calls() = json::array();
        rig.follower.requests = json::array();
        rig.follower.prepares = json::array();
        rig.follower.replies.clear();
        for (const auto& reply : op.value("responses", json::array())) {
            rig.follower.replies.push_back(reply);
        }
        json out = json::object();
        try {
            if (name == "init") {
                gw::FlightLinkOptions options;
                options.expected_uid = op.at("expected_uid");
                options.clock = [&now] { return now; };
                options.event_metadata = op.at("metadata").get<std::string>();
                options.command_owner_path = op.value("owner_path", "");
                profile_path = op.value("profile_path", "");
                options.profile_state_path = profile_path;
                options.link_factory = [&recorder](const std::string& address, int baud) {
                    recorder.current = std::make_shared<Channel>();
                    recorder.opened.push_back({address, baud});
                    return std::make_unique<FakeLink>(recorder.current, recorder);
                };
                options.new_id = make_id;
                if (op.value("reclaim", false)) {
                    gw::ReclaimController controller;
                    controller.present = [&controller_present] { return controller_present; };
                    controller.request = [&responses](const json& body, int timeout) {
                        calls().push_back({"rpc", body, timeout});
                        if (responses.empty()) {
                            throw gw::ValueError("추종기 응답 없음");
                        }
                        json reply = responses.front();
                        responses.pop_front();
                        return reply;
                    };
                    controller.wait = [gate](double seconds) { return gate->wait(seconds); };
                    controller.finished = [gate] { gate->finished(); };
                    link = std::make_unique<gw::ReclaimFlightLink>(std::move(options), controller,
                                                                   &authority);
                } else {
                    link = std::make_unique<gw::FlightLink>(std::move(options));
                }
                if (op.contains("owner")) {
                    const json& config = op.at("owner");
                    wire.tracks = config.value("tracks", false);
                    gw::FollowOwnerOptions owner_options;
                    owner_options.offboard_hover = config.value("offboard_hover", false);
                    owner_options.cv_gap_enabled = config.value("cv_gap", false);
                    owner_options.authority = &authority;
                    const std::scoped_lock<std::recursive_mutex> lock(link->mutex());
                    auto real = std::make_unique<gw::FollowBOwner>(
                        *link, std::make_unique<FakeOwnerTransport>(wire), owner_options);
                    owner = real.get();
                    link->state().follow_b = std::move(real);
                } else if (op.value("follow", false)) {
                    auto fake = std::make_unique<FakeFollow>();
                    follow = fake.get();
                    const std::scoped_lock<std::recursive_mutex> lock(link->mutex());
                    link->state().follow_b = std::move(fake);
                }
            } else if (name == "clock") {
                now = op.at("t").get<double>();
            } else if (name == "inject") {
                if (recorder.current && recorder.current->open) {
                    const gw::Bytes bytes = hex_bytes(op.at("hex"));
                    recorder.current->inbound.insert(recorder.current->inbound.end(), bytes.begin(),
                                                     bytes.end());
                }
            } else if (name == "step") {
                link->step();
            } else if (name == "connect") {
                link->request_connect(op.at("address"), op.at("baud"), op.at("client"));
            } else if (name == "submit") {
                gw::json_set(
                    out, "result",
                    link->submit(op.at("action"), op.at("client"), op.value("height", json())));
            } else if (name == "keepalive") {
                link->keepalive(op.at("client"), op.value("gs", false));
            } else if (name == "snapshot") {
                gw::json_set(out, "result", link->snapshot());
            } else if (name == "diagnostics") {
                gw::json_set(out, "result", link->diagnostics());
            } else if (name == "observations") {
                gw::json_set(out, "result", link->observations());
            } else if (name == "yaw_handoff") {
                gw::json_set(out, "result", link->yaw_handoff());
            } else if (name == "hold_owner") {
                holder = std::make_unique<gw::CommandOwner>(op.at("path").get<std::string>());
                gw::json_set(out, "result", holder->acquire());
            } else if (name == "release_owner") {
                holder.reset();
            } else if (name == "profile_file") {
                std::ifstream file(profile_path, std::ios::binary);
                gw::json_set(
                    out, "result",
                    file ? json(std::string(std::istreambuf_iterator<char>(file), {})) : json());
            } else if (name == "opened") {
                gw::json_set(out, "result", recorder.opened);
            } else if (name == "rpc") {
                wire.rpc_in.push_back(hex_parts(op.at("parts")));
            } else if (name == "sub") {
                wire.command_in.push_back(hex_parts(op.at("parts")));
            } else if (name == "track") {
                wire.track_in.push_back(hex_parts(op.at("parts")));
            } else if (name == "owner_state") {
                const std::scoped_lock<std::recursive_mutex> lock(link->mutex());
                gw::json_set(out, "result", json::parse(owner->debug_state().dump()));
            } else if (name == "manual_yaw") {
                const std::scoped_lock<std::recursive_mutex> lock(link->mutex());
                gw::json_set(out, "result", owner->manual_yaw().request(op.at("body")));
            } else if (name == "bench") {
                const std::scoped_lock<std::recursive_mutex> lock(link->mutex());
                gw::json_set(out, "result", owner->bench_request(op.at("body")));
            } else if (name == "follow_set") {
                const std::scoped_lock<std::recursive_mutex> lock(link->mutex());
                follow->token = op.value("token", follow->token);
                follow->bench = op.value("bench", follow->bench);
                follow->yaw.is_active = op.value("manual", follow->yaw.is_active);
                follow->rc_override_value = op.value("rc_override", follow->rc_override_value);
                follow->hover_enabled = op.value("hover_enabled", follow->hover_enabled);
                follow->hover_result = op.value("hover_result", follow->hover_result);
                follow->legacy = op.value("legacy", follow->legacy);
                if (op.contains("rate")) {
                    follow->yaw.rate = op.at("rate").is_null()
                                           ? std::optional<double>()
                                           : std::optional<double>(op.at("rate").get<double>());
                }
            } else if (name == "follow_get") {
                const std::scoped_lock<std::recursive_mutex> lock(link->mutex());
                gw::json_set(out, "result",
                             {{"ingests", follow->ingests},
                              {"ticks", follow->ticks},
                              {"rc_override", follow->rc_override_value},
                              {"bench", follow->bench}});
            } else if (name == "controller_set") {
                controller_present = op.value("present", controller_present);
                responses.clear();
                for (const auto& reply : op.value("responses", json::array())) {
                    responses.push_back(reply);
                }
            } else if (name == "reclaim") {
                auto* reclaim = dynamic_cast<gw::ReclaimFlightLink*>(link.get());
                {
                    const std::scoped_lock<std::mutex> lock(gate->mutex);
                    gate->active = true;
                }
                try {
                    gw::json_set(out, "result", reclaim->reclaim(op.at("client")));
                } catch (...) {
                    gate->finished();
                    throw;
                }
                gate->settle();
            } else if (name == "worker") {
                gw::json_set(out, "result", gate->advance());
            } else if (name == "bridge_init") {
                rig.preferences =
                    std::make_unique<gw::FollowPreferences>(op.at("prefs_path").get<std::string>());
                gw::ProfileServices services;
                services.prepare = [&rig](const std::string& uid, const gw::Wire& selection,
                                          bool reconfigure,
                                          const std::function<void()>& admission) {
                    rig.follower.prepares.push_back(
                        {uid, json::parse(selection.at("environment").dump()),
                         json::parse(selection.at("controller").dump()), reconfigure,
                         static_cast<bool>(admission)});
                    if (admission) {
                        admission();
                    }
                    return gw::Wire{
                        {"ok", true}, {"prepared", true}, {"reconfigured", reconfigure}};
                };
                services.status = [&rig](const std::string& uid) {
                    const gw::Wire result = rig.follower.forward(
                        gw::Wire{{"action", "status"}, {"expected_uid", uid}}, 350);
                    if (!(result.contains("accepted") && result.at("accepted") == true) ||
                        !gw::python_equal(gw::wire_get(result, "expected_uid"), gw::Wire(uid))) {
                        throw gw::ValueError("추종기 상태 확인 실패");
                    }
                    return result;
                };
                rig.profiles = std::make_unique<gw::FollowProfileCoordinator>(
                    *link, &authority, *rig.preferences, services);
                gw::MainGcsBridgeOptions options;
                options.monotonic = [&now] { return now; };
                options.monotonic_ns = [&now] {
                    return static_cast<std::int64_t>(std::llround(now * 1e9));
                };
                options.new_id = make_id;
                options.forward = [&rig](const gw::Wire& body, const std::string& /*uid*/,
                                         int timeout_ms) {
                    return rig.follower.forward(body, timeout_ms);
                };
                rig.log_path = op.at("log_path").get<std::string>();
                options.log_path = rig.log_path;
                options.router_gateway_port = op.at("router_gateway_port");
                options.uart_baud = op.at("uart_baud");
                // 대기 뒤에는 원본 FakeQuit.wait처럼 현재 종료 상태를 돌려준다
                options.wait = [&rig](double seconds) {
                    rig.worker_gate->wait(seconds);
                    return rig.bridge->quitting();
                };
                options.spawn = [](const std::function<void()>& body) { body(); };
                rig.bridge = std::make_unique<gw::MainGcsBridge>(
                    *link, authority, *rig.profiles, std::make_unique<FakeBridgeSockets>(rig.wire),
                    std::move(options));
                gw::MainGcsBridge* raw = rig.bridge.get();
                BridgeRig::start(rig.io_gate, rig.io_thread, [&rig, raw] {
                    while (!raw->quitting()) {
                        if (rig.io_gate->wait(-1.0) || raw->quitting()) {
                            break;
                        }
                        raw->io_once();
                    }
                    raw->io_finished();
                });
                BridgeRig::start(rig.worker_gate, rig.worker_thread, [&rig, raw] {
                    while (!raw->quitting()) {
                        if (rig.worker_gate->wait(-1.0) || raw->quitting()) {
                            break;
                        }
                        raw->worker_once();
                    }
                });
            } else if (name == "gui") {
                rig.wire.external.emplace_back(hex_bytes(op.at("hex")), op.at("host"),
                                               op.at("port"));
            } else if (name == "router_in") {
                rig.wire.router.push_back(hex_bytes(op.at("hex")));
            } else if (name == "tracks_in") {
                rig.wire.tracks.push_back(hex_parts(op.at("parts")));
            } else if (name == "router_fail") {
                rig.wire.router_fail = op.at("on").get<bool>();
            } else if (name == "router_refused") {
                rig.wire.router_refused = op.at("on").get<bool>();
            } else if (name == "bio") {
                gw::json_set(out, "result", rig.io_gate->advance());
            } else if (name == "bworker") {
                gw::json_set(out, "result", rig.worker_gate->advance());
            } else if (name == "bstate") {
                gw::json_set(out, "result", bridge_state(rig.bridge->debug_state()));
            } else if (name == "bprofiles") {
                gw::json_set(out, "result",
                             tagged(rig.profiles->snapshot(untagged(op.at("status")))));
            } else if (name == "bprefs_save") {
                gw::json_set(out, "result", tagged(rig.profiles->save(untagged(op.at("changes")))));
            } else if (name == "http_init") {
                const std::filesystem::path root = op.at("root").get<std::string>();
                const std::filesystem::path reference = op.at("reference").get<std::string>();
                rig.preferences = std::make_unique<gw::FollowPreferences>(
                    root / "config/operator/follow-preferences.json");
                gw::ProfileServices services;
                services.prepare = [&rig](const std::string& uid, const gw::Wire& selection,
                                          bool reconfigure,
                                          const std::function<void()>& admission) {
                    rig.follower.prepares.push_back(
                        {uid, json::parse(selection.at("environment").dump()),
                         json::parse(selection.at("controller").dump()), reconfigure,
                         static_cast<bool>(admission)});
                    if (admission) {
                        admission();
                    }
                    return gw::Wire{
                        {"ok", true}, {"prepared", true}, {"reconfigured", reconfigure}};
                };
                services.status = [&rig](const std::string& uid) {
                    const gw::Wire result = rig.follower.forward(
                        gw::Wire{{"action", "status"}, {"expected_uid", uid}}, 350);
                    if (!(result.contains("accepted") && result.at("accepted") == true) ||
                        !gw::python_equal(gw::wire_get(result, "expected_uid"), gw::Wire(uid))) {
                        throw gw::ValueError("추종기 상태 확인 실패");
                    }
                    return result;
                };
                rig.profiles = std::make_unique<gw::FollowProfileCoordinator>(
                    *link, &authority, *rig.preferences, services);
                const auto forward = [&rig](const gw::Wire& body, const std::string& /*uid*/,
                                            int timeout_ms) {
                    return rig.follower.forward(body, timeout_ms);
                };
                rig.maintenance = std::make_unique<gw::Maintenance>(
                    gw::MaintenanceOptions{reference, forward, "python3", 8768});
                const gw::OrderedJson settings =
                    gw::load_deployment_settings(reference, gw::current_environment());
                gw::GatewayApiOptions api_options;
                if (op.at("router_address").is_string()) {
                    api_options.router_address = op.at("router_address").get<std::string>();
                }
                api_options.uart_device = settings.at("device").at("uart_device");
                api_options.forward = forward;
                api_options.new_id = make_id;
                api_options.shutdown = [] {};
                std::array<char, 256> host{};
                ::gethostname(host.data(), host.size());
                api_options.host = host.data();
                api_options.serial_ports = gw::linux_serial_ports;
                rig.api =
                    std::make_unique<gw::GatewayApi>(*link, &authority, nullptr, *rig.profiles,
                                                     *rig.maintenance, std::move(api_options));
            } else if (name == "http") {
                gw::HttpRequest request;
                request.method = op.at("method");
                request.path = op.at("path");
                for (const auto& header : op.at("headers")) {
                    request.headers.emplace_back(header.at(0), header.at(1));
                }
                const std::string body = op.at("body").is_null() ? std::string() : [&] {
                    const gw::Bytes bytes = hex_bytes(op.at("body"));
                    return std::string(bytes.begin(), bytes.end());
                }();
                std::size_t offset = 0;
                request.read = [&body, &offset](std::size_t size) {
                    const std::string part = body.substr(offset, size);
                    offset += part.size();
                    return part;
                };
                json response = {{"status", nullptr}, {"body", nullptr}};
                try {
                    const gw::HttpResponse reply = rig.api->handle(request);
                    if (reply.send) {
                        response = {
                            {"status", reply.status},
                            {"body", reply.body.empty() ? json() : json::parse(reply.body)}};
                    }
                } catch (const std::exception&) {
                    response = {{"status", nullptr}, {"body", nullptr}};
                }
                gw::json_set(out, "response", response);
            } else if (name == "emergency") {
                authority.set_emergency(op.at("on").get<bool>());
            } else if (name == "authority") {
                const std::string method = op.at("method");
                const json client = op.value("client", json());
                const auto owner_json = [](const std::optional<std::string>& value) {
                    return value ? json(*value) : json();
                };
                const auto text = [&client](const char* message) {
                    if (!client.is_string()) {
                        throw gw::ValueError(message);
                    }
                    return client.get<std::string>();
                };
                try {
                    if (method == "admit") {
                        gw::json_set(out, "result",
                                     authority.admit(text("관제 세션 ID 오류"),
                                                     op.value("emergency", false)));
                    } else if (method == "release") {
                        authority.release(client.is_string() ? std::optional<std::string>(
                                                                   client.get<std::string>())
                                                             : std::nullopt,
                                          op.value("inactive", false));
                    } else if (method == "transfer") {
                        authority.transfer(text("관제 세션 ID 오류"), op.value("inactive", false));
                    } else if (method == "maintenance_claim") {
                        gw::json_set(
                            out, "result",
                            owner_json(authority.maintenance_claim(text("정비 GUI 세션 ID 오류"))));
                    } else if (method == "main_gui_claim") {
                        gw::json_set(
                            out, "result",
                            owner_json(authority.main_gui_claim(
                                text("메인 GUI 세션 ID 오류"), op.value("owner_live", false),
                                op.value("follow_active", false), op.value("fc_busy", false),
                                op.value("ground_disarmed", false))));
                    }
                } catch (const std::exception& error) {
                    out = error_json(error);
                }
                gw::json_set(out, "authority", authority.snapshot());
            }
        } catch (const std::exception& error) {
            out = error_json(error);
        }
        gw::json_set(out, "tx", recorder.frames);
        if (owner != nullptr) {
            gw::json_set(out, "published", wire.published);
            gw::json_set(out, "replies", wire.replies);
            wire.published = json::array();
            wire.replies = json::array();
        }
        gw::json_set(out, "calls", calls());
        calls() = json::array();
        gw::json_set(out, "worker", gate->state());
        // HTTP 시험은 추종기 요청만 낸다 (원본 실행기와 같은 항목)
        if (name == "http_init" || name == "http") {
            gw::json_set(out, "forward", rig.follower.requests);
            gw::json_set(out, "prepare", rig.follower.prepares);
        }
        // 브리지 항목은 5단계 시험(브리지를 만든 뒤)에서만 낸다 (2~4단계 출력은 그대로)
        if (rig.bridge) {
            gw::json_set(out, "gui_tx", rig.wire.gui_tx);
            gw::json_set(out, "router_tx", rig.wire.router_tx);
            rig.wire.gui_tx = json::array();
            rig.wire.router_tx = json::array();
            gw::json_set(out, "forward", rig.follower.requests);
            gw::json_set(out, "prepare", rig.follower.prepares);
            gw::json_set(out, "log", rig.log_lines());
            gw::json_set(out, "io_wait", rig.io_gate->state());
            gw::json_set(out, "worker_wait", rig.worker_gate->state());
        }
        emit(out);
    }
    rig.close();
    gate->close();
    return 0;
}

// 코덱 비교: 한 줄에 16진 바이트열, 해석한 메시지 목록을 낸다
int run_codec() {
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) {
            continue;
        }
        gw::MavlinkParser parser;
        parser.feed(hex_bytes(json::parse(line).at("hex")));
        json messages = json::array();
        while (auto message = parser.next()) {
            messages.push_back({{"kind", message->kind},
                                {"system", message->system},
                                {"component", message->component},
                                {"data", message->data}});
        }
        emit(messages);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv, std::next(argv, argc));
    try {
        if (args.size() >= 2 && args.at(1) == "vehicle") {
            return run_vehicle();
        }
        if (args.size() >= 2 && args.at(1) == "codec") {
            return run_codec();
        }
    } catch (const std::exception& error) {
        std::cerr << "probe 실패: " << error.what() << '\n';
        return 1;
    }
    std::cerr << "사용: argus_gateway_vehicle_probe vehicle|codec\n";
    return 2;
}
