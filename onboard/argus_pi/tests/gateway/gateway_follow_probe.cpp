// 게이트웨이 4a·5a단계 비교 도구.
// 4a: msgpack 코덱, CV 관측 기록, 추종기 요청, CV 진단 기록 (stage4a_diff.py)
// 5a: 대상 신원, 메인 GUI 추종 요청, 추종 설정·적용 판정 (stage5a_diff.py, bridge 모드)
// 6: 추종기·추적기 준비 (stage6b_diff.py, service 모드: 시험 폴더에서 가짜 프로세스를 실제로
// 띄운다) 표준 입력 JSON 줄을 실행하고 결과를 JSON 줄로 낸다. 비교 스크립트가 같은 입력을 Python
// 원본에 넣고 맞춘다. 네트워크를 열지 않는다 (추종기 전송은 가짜, 파일은 지정 폴더에만 쓴다).
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "argus/gateway/bridge/include/follow_preferences.hpp"
#include "argus/gateway/bridge/include/track_adapter.hpp"
#include "argus/gateway/bridge/include/track_identity.hpp"
#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/follow/include/cv_journal.hpp"
#include "argus/gateway/follow/include/cv_observation.hpp"
#include "argus/gateway/follow/include/follow_proxy.hpp"
#include "argus/gateway/service/include/follow_service.hpp"
#include "argus/gateway/service/include/preview_supervisor.hpp"

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

// 형식을 보존한 JSON 표현: 실수 {"$f": repr}, bytes {"$b": hex}, 사전 {"$m": [[키, 값], ...]}
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

json error_json(const std::exception& error) {
    const bool value_error = dynamic_cast<const gw::ValueError*>(&error) != nullptr;
    return {{"error", error.what()}, {"type", value_error ? "ValueError" : "Other"}};
}

void emit(const json& value) {
    std::cout << value.dump(-1, ' ', false, json::error_handler_t::replace) << '\n';
}

int run_msgpack() {
    std::string line;
    while (std::getline(std::cin, line)) {
        const json op = json::parse(line);
        json out = json::object();
        try {
            if (op.contains("hex")) {
                const gw::Wire value =
                    gw::msgpack_unpack(hex_bytes(op.at("hex")), op.value("strict", false));
                gw::json_set(out, "value", tagged(value));
                gw::json_set(out, "repack", gw::hex_lower(gw::msgpack_pack(value)));
            } else {
                gw::json_set(out, "packed",
                             gw::hex_lower(gw::msgpack_pack(untagged(op.at("value")))));
            }
        } catch (const std::exception& error) {
            out = error_json(error);
        }
        emit(out);
    }
    return 0;
}

int run_ledger() {
    gw::CvObservationLedger ledger;
    std::string line;
    while (std::getline(std::cin, line)) {
        const json op = json::parse(line);
        const std::string name = op.at("op");
        json out = json::object();
        if (name == "observe") {
            gw::json_set(
                out, "result",
                ledger.observe(untagged(op.at("frame")), op.at("now").get<std::int64_t>(),
                               op.at("epoch").get<std::int64_t>(), op.at("verified").get<bool>()));
        } else if (name == "contains" || name == "identity") {
            const gw::Wire generation = untagged(op.at("generation"));
            const gw::Wire track = untagged(op.at("track"));
            const gw::Wire stamp = untagged(op.at("stamp"));
            const auto epoch = op.at("epoch").get<std::int64_t>();
            const auto now = op.at("now").get<std::int64_t>();
            if (name == "contains") {
                gw::json_set(out, "result", ledger.contains(epoch, generation, track, stamp, now));
            } else {
                const auto who = ledger.identity(epoch, generation, track, stamp, now);
                gw::json_set(out, "result",
                             who ? json::array({who->source, who->category, who->label}) : json());
            }
        } else if (name == "clear") {
            ledger.clear();
        }
        gw::json_set(out, "state", tagged(ledger.debug_state()));
        emit(out);
    }
    return 0;
}

int run_proxy() {
    std::deque<json> script;
    std::int64_t now = 0;
    int next_id = 0;
    json sent = json::array();
    const auto transport = [&](const std::vector<gw::Bytes>& parts, int timeout) {
        json record = json::array();
        for (const auto& part : parts) {
            record.push_back(gw::hex_lower(part));
        }
        sent.push_back({{"parts", record}, {"timeout", timeout}});
        if (script.empty() || script.front().contains("error")) {
            if (!script.empty()) {
                script.pop_front();
            }
            throw gw::TransportError("timeout");
        }
        std::vector<gw::Bytes> reply;
        for (const auto& part : script.front().at("parts")) {
            reply.push_back(hex_bytes(part));
        }
        script.pop_front();
        return reply;
    };
    gw::FollowProxy proxy(
        "tcp://127.0.0.1:1", transport, [&now] { return now; },
        [&next_id] { return "id" + std::to_string(next_id++); });
    std::string line;
    while (std::getline(std::cin, line)) {
        const json op = json::parse(line);
        const std::string name = op.at("op");
        json out = json::object();
        sent = json::array();
        try {
            if (name == "script") {
                script.assign(op.at("replies").begin(), op.at("replies").end());
            } else if (name == "clock") {
                now = op.at("t").get<std::int64_t>();
            } else if (name == "forward") {
                gw::json_set(out, "result",
                             tagged(proxy.forward(untagged(op.at("body")), op.at("uid"),
                                                  op.value("timeout", 1200))));
            } else if (name == "source") {
                gw::json_set(out, "result", proxy.source());
            }
        } catch (const std::exception& error) {
            out = error_json(error);
        }
        gw::json_set(out, "sent", sent);
        emit(out);
    }
    return 0;
}

int run_journal(const std::string& directory) {
    {
        gw::CvJournal journal(directory.empty() ? std::nullopt
                                                : std::optional<std::filesystem::path>(directory));
        std::string line;
        while (std::getline(std::cin, line)) {
            const json op = json::parse(line);
            journal.record(op.at("event"), untagged(op.at("fields")));
        }
        journal.close();
        emit({{"path", journal.path() ? json(journal.path()->string()) : json()}});
    }
    return 0;
}

// ---------------------------------------------------------------- 5a: 메인 GUI 연결부 순수 함수
json file_state(const std::filesystem::path& folder, const std::filesystem::path& file) {
    json out = json::object();
    std::error_code error;
    if (std::filesystem::exists(file, error)) {
        std::ifstream stream(file, std::ios::binary);
        const std::string text((std::istreambuf_iterator<char>(stream)),
                               std::istreambuf_iterator<char>());
        gw::json_set(out, "file", gw::hex_lower(gw::Bytes(text.begin(), text.end())));
    } else {
        gw::json_set(out, "file", nullptr);
    }
    json names = json::array();
    if (std::filesystem::is_directory(folder, error)) {
        for (const auto& entry : std::filesystem::directory_iterator(folder)) {
            names.push_back(entry.path().filename().string());
        }
    }
    std::sort(names.begin(), names.end());
    gw::json_set(out, "names", names);
    return out;
}

std::string counter_id(int value) {
    std::string text = std::to_string(value);
    if (text.size() < 4) {
        text.insert(0, 4 - text.size(), '0');
    }
    return "id" + text;
}

void write_file(const std::filesystem::path& file, const gw::Bytes& data) {
    std::ofstream stream(file, std::ios::binary | std::ios::trunc);
    for (const std::uint8_t byte : data) {
        stream.put(static_cast<char>(byte));
    }
}

// 같은 op에 쓰는 가짜 추종기 (응답 목록을 차례로 돌려주고 보낸 요청을 남긴다)
struct FakeFollower {
    std::deque<json> replies;
    std::deque<bool> admissions;
    int next_id = 0;
    json sent = json::array();

    gw::Wire forward(const gw::Wire& body, const std::string& uid) {
        sent.push_back({{"body", tagged(body)}, {"uid", uid}});
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

    bool admit() {
        if (admissions.empty()) {
            return true;
        }
        const bool value = admissions.front();
        admissions.pop_front();
        return value;
    }
};

json run_bridge_op(const json& op, const std::filesystem::path& folder,
                   const std::filesystem::path& file, gw::FollowPreferences& preferences,
                   gw::HoldStability& hold, FakeFollower& fake) {
    const std::string name = op.at("op");
    const gw::ForwardRequest forward = [&fake](const gw::Wire& body, const std::string& uid) {
        return fake.forward(body, uid);
    };
    const gw::RequestIdSource new_id = [&fake] { return counter_id(fake.next_id++); };
    gw::Admission admission;
    if (op.contains("admission") && !op.at("admission").is_null()) {
        admission = [&fake] { return fake.admit(); };
    }
    json out = json::object();
    if (name == "identity") {
        gw::json_set(out, "result",
                     tagged(gw::resolve_track_start(untagged(op.at("params")),
                                                    untagged(op.at("candidates")))));
    } else if (name == "start") {
        gw::json_set(out, "result",
                     tagged(gw::track_start(untagged(op.at("params")), op.at("uid"), forward,
                                            admission, new_id)));
    } else if (name == "stop") {
        gw::json_set(out, "result", tagged(gw::track_stop(op.at("uid"), forward, new_id)));
    } else if (name == "search") {
        gw::json_set(out, "result",
                     tagged(gw::track_search(untagged(op.at("params")), op.at("uid"), forward,
                                             admission, new_id)));
    } else if (name == "scan_params") {
        gw::json_set(out, "result", tagged(gw::scan_parameters(untagged(op.at("params")))));
    } else if (name == "budget") {
        gw::json_set(out, "result",
                     tagged(gw::Wire(gw::scan_completion_budget_s(untagged(op.at("response")),
                                                                  untagged(op.at("requested"))))));
    } else if (name == "completion") {
        gw::json_set(out, "result",
                     tagged(gw::track_completion(
                         untagged(op.at("status")), untagged(op.at("command_id")),
                         untagged(op.at("controller_session")), untagged(op.at("onboard_session")),
                         untagged(op.at("fc_epoch")))));
    } else if (name == "env") {
        const std::string key = op.at("name");
        if (op.at("value").is_null()) {
            ::unsetenv(key.c_str());  // NOLINT(concurrency-mt-unsafe): 단일 스레드 도구
        } else {
            const std::string value = op.at("value");
            ::setenv(key.c_str(), value.c_str(), 1);  // NOLINT(concurrency-mt-unsafe): 위와 같음
        }
    } else if (name == "prefs_write") {
        std::error_code error;
        if (op.at("hex").is_null()) {
            std::filesystem::remove(file, error);
        } else {
            std::filesystem::create_directories(folder);
            write_file(file, hex_bytes(op.at("hex")));
        }
    } else if (name == "prefs_read") {
        gw::json_set(out, "result", tagged(preferences.read()));
    } else if (name == "prefs_save") {
        gw::json_set(out, "result", tagged(preferences.save(untagged(op.at("changes")))));
    } else if (name == "prefs_selected") {
        gw::json_set(out, "result", tagged(preferences.selected()));
    } else if (name == "safe") {
        gw::json_set(out, "result", gw::safe_to_prepare(untagged(op.at("state"))));
    } else if (name == "hold") {
        gw::json_set(out, "result",
                     hold.update(untagged(op.at("state")), untagged(op.at("sample")),
                                 op.at("now").get<double>(), untagged(op.at("identity"))));
    } else if (name == "hold_reset") {
        hold = gw::HoldStability();
    } else if (name == "active") {
        gw::json_set(out, "result", tagged(gw::active_profile(untagged(op.at("status")))));
    } else if (name == "verify") {
        gw::verify_runtime_environment(untagged(op.at("state")), untagged(op.at("status")));
        gw::json_set(out, "result", true);
    }
    return out;
}

int run_bridge(const std::string& directory) {
    const std::filesystem::path folder = std::filesystem::path(directory) / "config" / "operator";
    const std::filesystem::path file = folder / "follow-preferences.json";
    gw::FollowPreferences preferences(file);
    gw::HoldStability hold;
    FakeFollower fake;
    std::string line;
    while (std::getline(std::cin, line)) {
        const json op = json::parse(line);
        const std::string name = op.at("op");
        fake.sent = json::array();
        const json replies = op.value("replies", json::array());
        fake.replies.assign(replies.begin(), replies.end());
        fake.admissions.clear();
        if (op.contains("admission") && op.at("admission").is_array()) {
            for (const auto& value : op.at("admission")) {
                fake.admissions.push_back(value.get<bool>());
            }
        }
        json out;
        try {
            out = run_bridge_op(op, folder, file, preferences, hold, fake);
        } catch (const std::exception& error) {
            out = error_json(error);
        }
        if (name == "hold" || name == "hold_reset") {
            json samples = json::array();
            for (const auto& row : hold.samples()) {
                json values = json::array();
                for (const double value : row) {
                    values.push_back(tagged(gw::Wire(value)));
                }
                samples.push_back(values);
            }
            gw::json_set(out, "samples", samples);
            gw::json_set(out, "hold_identity", tagged(hold.identity()));
        }
        if (name.rfind("prefs", 0) == 0) {
            gw::json_set(out, "disk", file_state(folder, file));
        }
        gw::json_set(out, "sent", fake.sent);
        emit(out);
    }
    return 0;
}

// ---------------------------------------------------------------- 6: follow_service 준비
int run_service() {
    std::string line;
    while (std::getline(std::cin, line)) {
        const json op = json::parse(line);
        std::deque<json> replies(op.at("statuses").begin(), op.at("statuses").end());
        json last = replies.empty() ? json() : replies.back();
        json forwarded = json::array();
        int admissions = 0;
        gw::FollowServiceOptions options;
        options.root = op.at("root").get<std::string>();
        options.timeout_s = op.at("timeout").get<double>();
        options.forward = [&](const gw::Wire& body, const std::string& /*uid*/, int timeout_ms) {
            forwarded.push_back(json::array({tagged(body), timeout_ms}));
            json reply = last;
            if (!replies.empty()) {
                reply = replies.front();
                replies.pop_front();
            }
            if (reply.is_object() && reply.contains("$error")) {
                throw gw::ValueError(reply.at("$error").get<std::string>());
            }
            return untagged(reply);
        };
        const json& preview = op.at("preview");
        options.preview = [preview](const std::filesystem::path& /*root*/, double /*timeout*/) {
            if (preview.contains("error")) {
                throw gw::PreviewError(preview.at("error").get<std::string>());
            }
            return untagged(preview.at("value"));
        };
        std::function<void()> admission;
        if (!op.at("admission").is_null()) {
            const int raise_at = op.at("admission").value("raise_at", -1);
            admission = [&admissions, raise_at] {
                if (admissions++ == raise_at) {
                    throw gw::ValueError("지상 상태 재검사 실패");
                }
            };
        }
        gw::FollowService service(std::move(options));
        json out = json::object();
        try {
            gw::json_set(
                out, "result",
                tagged(service.prepare(op.at("uid"), op.at("environment"), op.at("controller"),
                                       op.at("reconfigure"), admission)));
        } catch (const std::exception& error) {
            out = error_json(error);
        }
        gw::json_set(out, "forward", forwarded);
        gw::json_set(out, "admissions", admissions);
        emit(out);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv, std::next(argv, argc));
    try {
        const std::string mode = args.size() >= 2 ? args.at(1) : "";
        if (mode == "msgpack") {
            return run_msgpack();
        }
        if (mode == "ledger") {
            return run_ledger();
        }
        if (mode == "proxy") {
            return run_proxy();
        }
        if (mode == "journal") {
            return run_journal(args.size() >= 3 ? args.at(2) : "");
        }
        if (mode == "bridge" && args.size() >= 3) {
            return run_bridge(args.at(2));
        }
        if (mode == "service") {
            return run_service();
        }
    } catch (const std::exception& error) {
        std::cerr << "probe 실패: " << error.what() << '\n';
        return 1;
    }
    std::cerr << "사용: argus_gateway_follow_probe msgpack|ledger|proxy|journal <dir>|bridge "
                 "<dir>|service\n";
    return 2;
}
