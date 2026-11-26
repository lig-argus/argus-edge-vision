// 게이트웨이 C++ 이식 비교 도구. 표준 입력의 JSON 줄(op)을 실행하고 결과를 JSON 줄로 낸다.
// tests/gateway/differential/*.py가 같은 입력을 Python 원본에 넣고 결과를 맞춰 본다.
// 장치·FC·네트워크를 열지 않는다 (owner 명령만 지정 파일에 flock을 건다).
#include <unistd.h>

#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "argus/gateway/events/include/event_log.hpp"
#include "argus/gateway/ownership/include/command_owner.hpp"
#include "argus/gateway/settings/include/deployment_settings.hpp"

using argus::gateway::Bytes;
using nlohmann::json;

namespace {

Bytes hex_bytes(const std::string& hex) {
    Bytes out;
    for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    }
    return out;
}

json error_json(const std::exception& error) {
    const bool value_error = dynamic_cast<const argus::gateway::SettingsError*>(&error) != nullptr;
    return {{"error", error.what()}, {"type", value_error ? "ValueError" : "Other"}};
}

int run_settings(const json& request) {
    argus::gateway::EnvironmentMap environment;
    const json variables =
        request.value("env", json::object());  // items()가 가리킬 객체를 살려 둔다
    for (const auto& [key, value] : variables.items()) {
        environment[key] = value.get<std::string>();
    }
    json out;
    try {
        auto data = argus::gateway::load_deployment_settings(request.at("root").get<std::string>(),
                                                             environment);
        if (request.value("materialize", false)) {
            data = argus::gateway::materialize_deployment_settings(std::move(data));
        }
        out = {{"ok", json::parse(data.dump())}};
    } catch (const std::exception& error) {
        out = error_json(error);
    }
    std::cout << out.dump() << '\n';
    return 0;
}

int run_events(const std::string& metadata) {
    double now = 0.0;
    argus::gateway::EventLog log([&now] { return now; }, metadata);
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) {
            continue;
        }
        const json op = json::parse(line);
        const std::string name = op.at("op");
        json out = json::object();
        if (name == "clock") {
            now = op.at("t").get<double>();
        } else if (name == "ingest") {
            log.ingest(op.at("kind"), op.at("data"));
        } else if (name == "due") {
            const auto due = log.due();
            out.emplace("due", due ? json::array({due->first, due->second}) : json());
        } else if (name == "verify") {
            log.verify(op.at("version"), op.at("info"));
        } else if (name == "boot") {
            out.emplace("reboot", log.observe_boot(op.at("ms").get<std::int64_t>()));
        } else if (name == "reset") {
            log.reset(op.at("reason"));
        } else if (name == "decode") {
            out = log.decode(op.at("id").get<std::uint32_t>(), hex_bytes(op.at("args_hex")));
        } else if (name == "snapshot") {
            out = log.snapshot();
        }
        if (op.value("state", false)) {
            out.emplace("_state", log.debug_state());
        }
        std::cout << out.dump() << '\n';
    }
    return 0;
}

// owner hold <path>: 잠금을 쥐고 "held"를 낸 뒤 표준 입력이 닫힐 때까지 기다린다.
int run_owner(const std::string& action, const std::string& path) {
    argus::gateway::CommandOwner owner(path);
    if (action == "status") {
        std::cout << owner.status().dump() << '\n';
        return 0;
    }
    const bool acquired = owner.acquire();
    std::cout << json{{"acquired", acquired}, {"status", owner.status()}}.dump() << '\n'
              << std::flush;
    if (action == "hold" && acquired) {
        std::string line;
        while (std::getline(std::cin, line)) {
        }
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv, std::next(argv, argc));
    if (args.size() < 2) {
        std::cerr << "사용: argus_gateway_probe settings|events <metadata>|owner "
                     "<acquire|hold|status> <path>\n";
        return 2;
    }
    const std::string& command = args.at(1);
    try {
        if (command == "settings") {
            const std::string text((std::istreambuf_iterator<char>(std::cin)),
                                   std::istreambuf_iterator<char>());
            return run_settings(json::parse(text));
        }
        if (command == "events" && args.size() >= 3) {
            return run_events(args.at(2));
        }
        if (command == "owner" && args.size() >= 4) {
            return run_owner(args.at(2), args.at(3));
        }
    } catch (const std::exception& error) {
        std::cerr << "probe 실패: " << error.what() << '\n';
        return 1;
    }
    std::cerr << "알 수 없는 명령\n";
    return 2;
}
