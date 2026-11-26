#include "argus/gateway/settings/include/deployment_settings.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <regex>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

#include "argus/gateway/common/include/python_compat.hpp"
#include "argus/gateway/common/include/sha256.hpp"

namespace argus::gateway {

namespace fs = std::filesystem;

namespace {

// 원본 ENV_KEYS 순서 (오류가 나면 먼저 검사한 키 이름이 메시지에 들어간다)
// 2026-11-25 batch 4: vision (팀 6374, argus-onboard 가 bind) 가 예전 preview_input (5555),
// preview_output (5558) 를 대신한다
constexpr std::array<std::pair<const char*, const char*>, 6> kEndpointKeys{{
    {"fc_state", "ARGUS_FOLLOW_B_FC_STATE_ENDPOINT"},
    {"mode_rpc", "ARGUS_FOLLOW_B_ONBOARD_RPC_ENDPOINT"},
    {"control", "ARGUS_FOLLOW_B_CONTROL_ENDPOINT"},
    {"operator", "ARGUS_FOLLOW_B_OPERATOR_ENDPOINT"},
    {"vision", "ARGUS_VISION_ENDPOINT"},
    {"tracks", "ARGUS_TRACKS_ENDPOINT"},
}};

// 원본 RENAMED_ENV: 예전 이름이 환경에 남아 있으면 받지 않는다
constexpr std::array<std::pair<const char*, const char*>, 2> kRenamedVariables{{
    {"ARGUS_BUS_INPUT_ENDPOINT", "ARGUS_VISION_ENDPOINT"},
    {"ARGUS_PREVIEW_ENDPOINT", "ARGUS_VISION_ENDPOINT"},
}};

// 원본 RETIRED_OBSERVATION_PORTS: 예전 관측 포트 값은 받지 않는다 (2026-11-25 batch 4 리뷰)
constexpr std::array<int, 3> kRetiredObservationPorts{5555, 5556, 5558};

// load가 읽는 나머지 환경 변수
constexpr std::array<const char*, 6> kOtherVariables{
    "ARGUS_FOLLOW_ENVIRONMENT",      "ARGUS_FOLLOW_CONTROLLER", "ARGUS_OBSERVATIONS_ENDPOINT",
    "ARGUS_FOLLOW_B_CAMERA_PROFILE", "ARGUS_BIN_DIR",           "ARGUS_FOLLOW_B_BUILD_DIR"};

// Python dict[key] = value: 있는 키는 자리를 지키고 새 키는 끝에 붙는다
void set_key(OrderedJson& object, const std::string& key, OrderedJson value) {
    if (object.contains(key)) {
        object.at(key) = std::move(value);
    } else {
        object.emplace(key, std::move(value));
    }
}

std::string read_bytes(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("FileNotFoundError: " + path.string());
    }
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

// Python read_text(): 범용 줄바꿈(\r\n, \r → \n)
std::string read_text_universal(const fs::path& path) {
    const std::string raw = read_bytes(path);
    std::string out;
    out.reserve(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i) {
        if (raw.at(i) == '\r') {
            out += '\n';
            if (i + 1 < raw.size() && raw.at(i + 1) == '\n') {
                ++i;
            }
        } else {
            out += raw.at(i);
        }
    }
    return out;
}

OrderedJson read_json(const fs::path& path) {
    return OrderedJson::parse(read_text_universal(path));
}

// Python str.splitlines()
std::vector<std::string> split_lines(const std::string& text) {
    const std::u32string wide = utf8_to_u32(text);
    std::vector<std::string> lines;
    std::u32string current;
    for (std::size_t i = 0; i < wide.size(); ++i) {
        const char32_t c = wide.at(i);
        const bool separator = c == U'\n' || c == U'\r' || c == 0x0b || c == 0x0c || c == 0x1c ||
                               c == 0x1d || c == 0x1e || c == 0x85 || c == 0x2028 || c == 0x2029;
        if (!separator) {
            current += c;
            continue;
        }
        if (c == U'\r' && i + 1 < wide.size() && wide.at(i + 1) == U'\n') {
            ++i;
        }
        lines.push_back(u32_to_utf8(current));
        current.clear();
    }
    if (!current.empty()) {
        lines.push_back(u32_to_utf8(current));
    }
    return lines;
}

// Python str(value)
std::string python_str(const OrderedJson& value) {
    if (value.is_string()) {
        return value.get<std::string>();
    }
    if (value.is_boolean()) {
        return value.get<bool>() ? "True" : "False";
    }
    if (value.is_null()) {
        return "None";
    }
    if (value.is_number_float()) {
        return python_float_repr(value.get<double>());
    }
    if (value.is_number_unsigned()) {
        return std::to_string(value.get<std::uint64_t>());
    }
    if (value.is_number_integer()) {
        return std::to_string(value.get<std::int64_t>());
    }
    throw std::runtime_error("TypeError: str() 변환 미지원");
}

// Python type(value) is int and low <= value <= high
bool int_in_range(const OrderedJson& value, long long low, long long high) {
    if (value.is_number_unsigned()) {
        const auto number = value.get<std::uint64_t>();
        return number <= static_cast<std::uint64_t>(high) && static_cast<long long>(number) >= low;
    }
    if (value.is_number_integer()) {
        const auto number = value.get<std::int64_t>();
        return number >= low && number <= high;
    }
    return false;
}

std::string require_string(const OrderedJson& value) {
    if (!value.is_string()) {
        throw std::runtime_error("TypeError: 문자열이 필요합니다");
    }
    return value.get<std::string>();
}

const std::regex& loopback_endpoint() {
    static const std::regex kPattern(R"(tcp://127\.0\.0\.1:[0-9]+)");
    return kPattern;
}

bool endpoint_port_ok(const std::string& value) {
    const std::string digits = value.substr(value.rfind(':') + 1);
    const std::size_t start = digits.find_first_not_of('0');
    if (start == std::string::npos) {
        return false;  // 0
    }
    if (digits.size() - start > 6) {
        return false;
    }
    const long port = std::stol(digits.substr(start));
    return port >= 1024 && port <= 65535;
}

bool valid_endpoint(const std::string& value) {
    return std::regex_match(value, loopback_endpoint()) && endpoint_port_ok(value);
}

fs::path python_resolve(const fs::path& path) { return fs::weakly_canonical(fs::absolute(path)); }

bool is_relative_to(const fs::path& path, const fs::path& root) {
    auto it = path.begin();
    for (const auto& part : root) {
        if (it == path.end() || *it != part) {
            return false;
        }
        ++it;
    }
    return true;
}

std::string env_or(const EnvironmentMap& environment, const std::string& key,
                   const std::string& fallback) {
    const auto found = environment.find(key);
    return found == environment.end() ? fallback : found->second;
}

std::string env_truthy(const EnvironmentMap& environment, const std::string& key) {
    const auto found = environment.find(key);
    return found == environment.end() ? std::string() : found->second;
}

OrderedJson load_v21(const OrderedJson& data) {
    OrderedJson profile = OrderedJson::object();
    set_key(profile, "schema", "argus.follow.v2_1.v1");
    set_key(profile, "enabled", false);
    set_key(profile, "box_motion_guard", false);
    set_key(profile, "cv_gap", false);
    set_key(profile, "cv_visibility", false);
    set_key(profile, "vertical_center", false);
    set_key(profile, "vertical_center_weight", 0.5);
    set_key(profile, "cv_gap_max_s", 3.0);
    set_key(profile, "receipt_retry_ms", 0);
    set_key(profile, "frame_diagnostics", false);
    const OrderedJson& paths = data.at("paths");
    const auto found = paths.find("v2_1");
    if (found != paths.end() && python_truthy(nlohmann::json::parse(found->dump()))) {
        const OrderedJson values = read_json(require_string(*found));
        if (!values.is_object()) {
            throw SettingsError("Unknown V2.1 profile fields");
        }
        for (const auto& item : values.items()) {
            if (!profile.contains(item.key())) {
                throw SettingsError("Unknown V2.1 profile fields");
            }
        }
        for (const auto& item : values.items()) {
            set_key(profile, item.key(), item.value());
        }
    }
    if (profile.at("schema") != OrderedJson("argus.follow.v2_1.v1")) {
        throw SettingsError("Invalid V2.1 feature schema");
    }
    for (const char* key : {"enabled", "box_motion_guard", "cv_gap", "cv_visibility",
                            "vertical_center", "frame_diagnostics"}) {
        if (!profile.at(key).is_boolean()) {
            throw SettingsError(std::string("V2.1 feature must be boolean: ") + key);
        }
    }
    const OrderedJson& weight = profile.at("vertical_center_weight");
    const OrderedJson& gap = profile.at("cv_gap_max_s");
    if (!int_in_range(profile.at("receipt_retry_ms"), 0, 50) || !weight.is_number() ||
        weight.get<double>() != 0.5 || !gap.is_number() || gap.get<double>() != 3.0) {
        throw SettingsError("V2.1 numeric contract differs from the bounded 3s/.5 profile");
    }
    return profile;
}

void atomic_write(const fs::path& path, const std::string& content) {
    fs::create_directories(path.parent_path());
    if (fs::is_regular_file(path) && read_text_universal(path) == content) {
        return;
    }
    std::string name = (path.parent_path() / (path.filename().string() + ".XXXXXX")).string();
    std::vector<char> buffer(name.begin(), name.end());
    buffer.push_back('\0');
    const int fd = ::mkstemp(buffer.data());
    if (fd < 0) {
        throw std::runtime_error("OSError: 임시 파일 생성 실패 " + path.string());
    }
    const std::string temporary(buffer.data());
    bool ok = true;
    std::size_t written = 0;
    while (written < content.size()) {
        const ssize_t count =
            ::write(fd, std::next(content.data(), static_cast<std::ptrdiff_t>(written)),
                    content.size() - written);
        if (count <= 0) {
            ok = false;
            break;
        }
        written += static_cast<std::size_t>(count);
    }
    ok = ok && ::fsync(fd) == 0;
    ok = (::close(fd) == 0) && ok;
    if (!ok || ::rename(temporary.c_str(), path.c_str()) != 0) {
        ::unlink(temporary.c_str());
        throw std::runtime_error("OSError: 설정 파일 쓰기 실패 " + path.string());
    }
}

}  // namespace

EnvironmentMap current_environment() {
    EnvironmentMap out;
    const auto add = [&out](const char* name) {
        const char* value = std::getenv(name);
        if (value != nullptr) {
            out.emplace(name, value);
        }
    };
    for (const auto& item : kEndpointKeys) {
        add(item.second);
    }
    for (const char* name : kOtherVariables) {
        add(name);
    }
    for (const auto& item : kRenamedVariables) {
        add(item.first);
    }
    return out;
}

OrderedJson load_deployment_settings(const fs::path& root_input,
                                     const EnvironmentMap& environment) {
    const fs::path root = python_resolve(root_input);
    OrderedJson data = read_json(root / "config/runtime.json");
    const std::string flight_environment =
        env_or(environment, "ARGUS_FOLLOW_ENVIRONMENT", "indoor");
    if (flight_environment != "indoor" && flight_environment != "outdoor") {
        throw SettingsError("Unknown follow environment");
    }
    set_key(data, "flight_environment", flight_environment);
    const std::string controller = env_or(environment, "ARGUS_FOLLOW_CONTROLLER", "predictive");
    const bool mount30 = controller == "predictive_m30" || controller == "predictive_low_m30";
    const bool low = controller == "predictive_low" || controller == "predictive_low_m30";
    // 2026-11-12 실외: 45° 장착 4 m·3 m, 30° 장착(지평면 아래) 4 m·3 m
    // 2026-11-18 데모 모드: demo는 실내 전용 (회전 추종만, 고도 1.5 m 유지)
    const bool demo = controller == "demo";
    // 2026-11-19 데모2: demo2는 실내 전용 (예측 이동 추종, yaw 고정)
    const bool demo2 = controller == "demo2";
    if ((controller != "predictive" && controller != "predictive_low" && !mount30 && !demo &&
         !demo2) ||
        (flight_environment == "indoor" && controller != "predictive" && !demo && !demo2) ||
        (flight_environment != "indoor" && (demo || demo2))) {
        throw SettingsError("Unknown follow profile for the selected environment");
    }
    set_key(data, "follow_controller", controller);
    if (demo) {
        // 이동, 회전 두 자리 모두 같은 회전 전용 설정을 쓴다. 측정/카메라는 실내 기본값.
        OrderedJson& paths = data.at("paths");
        set_key(paths, "moving", "config/follow/demo-yaw.config");
        set_key(paths, "yaw", "config/follow/demo-yaw.config");
    }
    if (demo2) {
        // 이동 칸은 yaw 고정 예측 추종, 회전 칸은 실내 회전 설정의 demo2 이름판. 측정/카메라는 실내
        // 기본값.
        OrderedJson& paths = data.at("paths");
        set_key(paths, "moving", "config/follow/demo2-moving.config");
        set_key(paths, "yaw", "config/follow/demo2-yaw.config");
    }
    if (flight_environment == "outdoor") {
        OrderedJson& paths = data.at("paths");
        const std::string prefix =
            std::string("config/follow/outdoor-") + (low ? "low-" : "") + (mount30 ? "m30-" : "");
        set_key(paths, "moving", prefix + "predictive-moving.config");
        set_key(paths, "yaw", prefix + "yaw.config");
        set_key(paths, "measurement",
                mount30 ? "config/tracking/measurement-outdoor-m30.json"
                        : "config/tracking/measurement-outdoor.json");
        if (mount30) {
            set_key(paths, "camera", "config/device/camera-mount30.json");
        }
    }
    const OrderedJson device = read_json(root / "config/device/device.json");
    const OrderedJson& uid = device.at("expected_fc_uid");
    static const std::regex kUidPattern("[0-9a-fA-F]{36}");
    if (!uid.is_string() || !std::regex_match(uid.get<std::string>(), kUidPattern)) {
        throw SettingsError("Invalid device expected_fc_uid; no implicit identity fallback");
    }
    for (const char* key : {"expected_fc_system_id", "expected_fc_component_id"}) {
        if (!int_in_range(device.at(key), 1, 255)) {
            throw SettingsError(std::string("Invalid device ") + key);
        }
    }
    static const std::regex kUartPattern(R"(/dev/(?:tty(?:AMA|USB|ACM|S)[0-9]+|serial[01]))");
    if (!std::regex_match(require_string(device.at("uart_device")), kUartPattern)) {
        throw SettingsError("Invalid UART device");
    }
    if (!int_in_range(device.at("uart_baud"), 1, std::numeric_limits<long long>::max())) {
        throw SettingsError("Invalid UART baud");
    }
    for (const char* key : {"router_gateway_port", "router_legacy_port"}) {
        if (!int_in_range(device.at(key), 1024, 65535)) {
            throw SettingsError("Invalid router port");
        }
    }
    if (device.at("router_gateway_port") == device.at("router_legacy_port")) {
        throw SettingsError("Router port conflict");
    }
    OrderedJson& endpoints = data.at("endpoints");
    for (const auto& [variable, replacement] : kRenamedVariables) {
        if (environment.find(variable) != environment.end()) {
            throw SettingsError(std::string("Renamed endpoint variable: ") + variable + " -> " +
                                replacement);
        }
    }
    for (const auto& [key, variable] : kEndpointKeys) {
        const OrderedJson fallback = endpoints.at(key);
        const auto found = environment.find(variable);
        const std::string value =
            found != environment.end() ? found->second : require_string(fallback);
        if (!std::regex_match(value, loopback_endpoint())) {
            throw SettingsError(std::string("Endpoint must be explicit loopback TCP: ") + key);
        }
        if (!endpoint_port_ok(value)) {
            throw SettingsError(std::string("Invalid endpoint port: ") + key);
        }
        endpoints.at(key) = value;
    }
    std::set<std::string> unique;
    for (const auto& item : endpoints.items()) {
        unique.insert(item.value().dump());
    }
    if (unique.size() != endpoints.size()) {
        throw SettingsError("Endpoint ownership conflict");
    }
    const std::string observation =
        env_or(environment, "ARGUS_OBSERVATIONS_ENDPOINT", require_string(endpoints.at("vision")));
    if (!valid_endpoint(observation)) {
        throw SettingsError("Invalid observation endpoint");
    }
    const int observation_port = std::stoi(observation.substr(observation.rfind(':') + 1));
    if (std::find(kRetiredObservationPorts.begin(), kRetiredObservationPorts.end(),
                  observation_port) != kRetiredObservationPorts.end()) {
        throw SettingsError(
            "Retired endpoint value: ARGUS_OBSERVATIONS_ENDPOINT -> ARGUS_VISION_ENDPOINT");
    }
    set_key(data, "observation_endpoint", observation);
    OrderedJson& paths = data.at("paths");
    for (const auto& item : paths.items()) {
        const fs::path resolved = python_resolve(root / require_string(item.value()));
        if (!is_relative_to(resolved, root) || !fs::is_regular_file(resolved)) {
            throw SettingsError("Missing/bad deployment path: " + item.key());
        }
        item.value() = resolved.string();
    }
    const std::string camera_override = env_truthy(environment, "ARGUS_FOLLOW_B_CAMERA_PROFILE");
    if (!camera_override.empty()) {
        set_key(paths, "camera", fs::canonical(camera_override).string());
    }
    const OrderedJson camera = read_json(require_string(paths.at("camera")));
    if (camera.at("expected_uid") != uid) {
        throw SettingsError("Camera calibration/device UID mismatch; recalibration required");
    }
    set_key(data, "v2_1", load_v21(data));
    set_key(data, "device", device);
    set_key(data, "root", root.string());
    const std::string modern_bin = env_truthy(environment, "ARGUS_BIN_DIR");
    const std::string legacy_bin = env_truthy(environment, "ARGUS_FOLLOW_B_BUILD_DIR");
    if (!modern_bin.empty() && !legacy_bin.empty() &&
        python_resolve(modern_bin) != python_resolve(legacy_bin)) {
        throw SettingsError("Conflicting runtime binary directories");
    }
    fs::path binary = root / "bin";
    if (!modern_bin.empty()) {
        binary = modern_bin;
    } else if (!legacy_bin.empty()) {
        binary = legacy_bin;
    }
    set_key(data, "binary_dir", python_resolve(binary).string());
    return data;
}

OrderedJson materialize_deployment_settings(OrderedJson data) {
    const std::string digest =
        sha256_hex(python_json_dumps_sorted(nlohmann::json::parse(data.dump()))).substr(0, 16);
    const fs::path output = fs::path(require_string(data.at("root"))) / "run/config" / digest;
    const OrderedJson& device = data.at("device");
    const OrderedJson& endpoints = data.at("endpoints");
    const std::vector<std::pair<std::string, std::string>> values{
        {"expected_fc_uid", python_str(device.at("expected_fc_uid"))},
        {"expected_fc_system_id", python_str(device.at("expected_fc_system_id"))},
        {"expected_fc_component_id", python_str(device.at("expected_fc_component_id"))},
        {"tracks_sub_endpoint", python_str(endpoints.at("tracks"))},
        {"tracks_pub_endpoint", python_str(endpoints.at("tracks"))},
    };
    for (const char* mode : {"moving", "yaw"}) {
        std::vector<std::string> lines;
        for (std::string& line :
             split_lines(read_text_universal(require_string(data.at("paths").at(mode))))) {
            const std::string key = line.substr(0, line.find('='));
            bool replaced = false;
            for (const auto& value : values) {
                replaced = replaced || value.first == key;
            }
            if (!replaced) {
                lines.push_back(std::move(line));
            }
        }
        for (const auto& [key, value] : values) {
            std::string line = key;
            line += '=';
            line += value;
            lines.push_back(std::move(line));
        }
        std::string content;
        for (const std::string& line : lines) {
            content += line;
            content += '\n';  // '\n'.join(lines) + '\n'
        }
        const fs::path path = output / (std::string(mode) + ".config");
        atomic_write(path, content);
        data.at("paths").at(mode) = path.string();
    }
    for (const char* key : {"router_gateway_port", "router_legacy_port"}) {
        const OrderedJson& port = device.at(key);
        if (port == OrderedJson(14550) || port == OrderedJson(14561)) {
            throw SettingsError("Configured router port conflicts with the common GCS bridge");
        }
    }
    std::ostringstream router;
    router << "[General]\nTcpServerPort = 0\nReportStats = false\n\n"
           << "[UartEndpoint fc]\nDevice = " << python_str(device.at("uart_device"))
           << "\nBaud = " << python_str(device.at("uart_baud")) << "\nFlowControl = false\n\n"
           << "[UdpEndpoint gui]\nMode = Server\nAddress = 127.0.0.1\nPort = "
           << python_str(device.at("router_gateway_port")) << "\n\n"
           << "[UdpEndpoint yaw]\nMode = Normal\nAddress = 127.0.0.1\nPort = "
           << python_str(device.at("router_legacy_port")) << "\n\n"
           << "[UdpEndpoint main_gcs_proxy]\nMode = Server\nAddress = 127.0.0.1\nPort = 14561\n";
    atomic_write(output / "mavlink-router.conf", router.str());
    set_key(data, "router_config", (output / "mavlink-router.conf").string());
    return data;
}

}  // namespace argus::gateway
