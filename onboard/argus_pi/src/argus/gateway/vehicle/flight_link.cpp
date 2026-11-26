#include "argus/gateway/vehicle/include/flight_link.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <regex>
#include <set>
#include <stdexcept>
#include <vector>

#include "argus/gateway/common/include/python_compat.hpp"

namespace argus::gateway {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// pymavlink 상수 (원본 vehicle.M과 같은 값)
constexpr std::int64_t kSafetyArmed = 128;
constexpr std::int64_t kLandedOnGround = 1;
constexpr std::int64_t kLandedInAir = 2;
constexpr std::int64_t kLandedTakeoff = 3;
constexpr std::int64_t kLandedLanding = 4;
constexpr std::int64_t kPrearmCheck = 268435456;
constexpr std::int64_t kAbsolutePressure = 8;
constexpr int kParamReal32 = 9;
constexpr int kParamInt32 = 6;
constexpr int kParamUint32 = 5;
constexpr std::uint16_t kCmdDoSetMode = 176;
constexpr std::uint16_t kCmdArmDisarm = 400;
constexpr std::uint16_t kCmdNavLand = 21;
constexpr std::uint16_t kCmdDoSetHome = 179;
constexpr std::uint16_t kCmdReboot = 246;
constexpr std::uint16_t kCmdRequestMessage = 512;
constexpr std::uint16_t kCmdSetMessageInterval = 511;
constexpr std::int64_t kAutopilotPx4 = 12;
constexpr std::int64_t kPitch270 = 25;
constexpr std::int64_t kParamEncodeBytewise = 16;
constexpr std::int64_t kParamEncodeCCast = 131072;
constexpr std::int64_t kResultAccepted = 0;
constexpr std::int64_t kResultInProgress = 5;
constexpr double kMsgHomePosition = 242;
constexpr double kMsgAutopilotVersion = 148;
constexpr double kMsgComponentInformation = 395;
constexpr std::uint8_t kTypeOnboardController = 18;
constexpr std::uint8_t kTypeGcs = 6;
constexpr std::uint8_t kAutopilotInvalid = 8;
constexpr std::uint8_t kStateActive = 4;
constexpr std::uint8_t kFrameLocalNed = 1;
constexpr std::uint16_t kZeroMask = 1479;
constexpr std::uint16_t kYawIgnore = 1024;
constexpr std::uint16_t kYawRateIgnore = 2048;
// 2026-11-18 이륙 조건 보강: FC 강한 자기장 간섭 경고 거부 문구
constexpr const char* kMagArmRefusal =
    "FC 강한 자기장 간섭 경고 - 방향(yaw) 정렬이 안 될 수 있습니다. "
    "이륙 위치를 옮기고 경고가 사라진 뒤 ARM하세요";
constexpr const char* kMagTakeoffRefusal =
    "ARM 직전 FC 강한 자기장 간섭 경고 - 착륙 해제(disarm) 후 위치를 옮겨 다시 시도하세요";
// VX|VY|VZ|AX|AY|AZ|YAW_RATE 무시
constexpr std::uint16_t kPositionMask = 8 | 16 | 32 | 64 | 128 | 256 | 2048;

const std::array<const char*, 24> kProfileParams{
    "SYS_HAS_GPS",      "SYS_HAS_MAG",    "EKF2_GPS_CTRL",    "EKF2_OF_CTRL",
    "EKF2_RNG_CTRL",    "EKF2_HGT_REF",   "EKF2_BARO_CTRL",   "SENS_FLOW_MAXHGT",
    "SENS_FLOW_MINHGT", "SYS_HAS_NUM_OF", "SYS_HAS_NUM_DIST", "GF_MAX_VER_DIST",
    "MIS_TAKEOFF_ALT",  "NAV_DLL_ACT",    "COM_RC_IN_MODE",   "COM_ARM_WO_GPS",
    "RC_MAP_THROTTLE",  "EKF2_OF_QMIN",   "EKF2_OF_QMIN_GND", "COM_DISARM_PRFLT",
    "COM_OF_LOSS_T",    "COM_OBL_RC_ACT", "MPC_TKO_SPEED",    "MPC_THR_HOVER"};
const std::array<const char*, 1> kProfileParamsExtra{"COM_POS_FS_EPH"};

// 원본 STREAMS (dict 순서)
constexpr std::array<std::pair<int, double>, 17> kStreams{{{1, 2},
                                                           {24, 2},
                                                           {30, 60},
                                                           {32, 50},
                                                           {33, 5},
                                                           {65, 5},
                                                           {106, 20},
                                                           {132, 50},
                                                           {141, 10},
                                                           {147, 2},
                                                           {230, 5},
                                                           {242, 1},
                                                           {245, 2},
                                                           {83, 5},
                                                           {85, 5},
                                                           {36, 5},
                                                           {2, 1}}};

constexpr std::array<std::pair<const char*, std::pair<int, int>>, 7> kModes{{
    {"position", {3, 0}},
    {"altitude", {2, 0}},
    {"stabilized", {7, 0}},
    {"hold", {4, 3}},
    {"takeoff", {4, 2}},
    {"land", {4, 6}},
    {"offboard", {6, 0}},
}};

const std::array<const char*, 17> kSnapshotActions{
    "arm",      "disarm",         "takeoff",         "hold",           "land",   "set_home",
    "offboard", "indoor_profile", "outdoor_profile", "height_profile", "manual", "position",
    "altitude", "stabilized",     "disconnect",      "set_height",     "reboot"};

// 원본 M.enums["MAV_RESULT"] (ardupilotmega dialect: 6 없음, 9는 ENUM_END)
std::string result_name(std::int64_t result) {
    static const std::map<std::int64_t, const char*> kNames{{0, "MAV_RESULT_ACCEPTED"},
                                                            {1, "MAV_RESULT_TEMPORARILY_REJECTED"},
                                                            {2, "MAV_RESULT_DENIED"},
                                                            {3, "MAV_RESULT_UNSUPPORTED"},
                                                            {4, "MAV_RESULT_FAILED"},
                                                            {5, "MAV_RESULT_IN_PROGRESS"},
                                                            {7, "MAV_RESULT_COMMAND_LONG_ONLY"},
                                                            {8, "MAV_RESULT_COMMAND_INT_ONLY"},
                                                            {9, "MAV_RESULT_ENUM_END"}};
    const auto found = kNames.find(result);
    return found == kNames.end() ? std::to_string(result) : std::string(found->second);
}

bool is_profile_param(const std::string& name) {
    return std::any_of(kProfileParams.begin(), kProfileParams.end(),
                       [&name](const char* item) { return name == item; }) ||
           name == kProfileParamsExtra.at(0);
}

// Python KeyError 문구 ('key')
[[noreturn]] void key_error(const std::string& key) { throw std::runtime_error("'" + key + "'"); }

const Json& at_key(const Json& object, const std::string& key) {
    if (!object.is_object()) {
        throw std::runtime_error("'NoneType' object is not subscriptable");
    }
    const auto found = object.find(key);
    if (found == object.end()) {
        key_error(key);
    }
    return *found;
}

// Python dict.get(key) (없으면 None)
Json get_or_null(const Json* object, const std::string& key) {
    if (object == nullptr || !object->is_object()) {
        return nullptr;
    }
    const auto found = object->find(key);
    return found == object->end() ? Json() : *found;
}

std::int64_t as_int(const Json& value) {
    if (value.is_number_unsigned()) {
        return static_cast<std::int64_t>(value.get<std::uint64_t>());
    }
    if (value.is_number_integer()) {
        return value.get<std::int64_t>();
    }
    if (value.is_number_float()) {
        return static_cast<std::int64_t>(value.get<double>());
    }
    if (value.is_boolean()) {
        return value.get<bool>() ? 1 : 0;
    }
    throw std::runtime_error(
        "int() argument must be a string, a bytes-like object or a real number");
}

double as_double(const Json& value) {
    if (value.is_number() || value.is_boolean()) {
        return value.is_boolean() ? static_cast<double>(value.get<bool>()) : value.get<double>();
    }
    throw std::runtime_error("must be real number");
}

Json optional_number(const std::optional<double>& value) { return value ? Json(*value) : Json(); }

// Python == (숫자끼리는 값 비교, None은 None과만 같다)
bool python_equal(const Json& left, double right) {
    return (left.is_number() || left.is_boolean()) && as_double(left) == right;
}

// 원본 dict(INDOOR_PROFILE) 값: 실내·실외는 정수, 고도 프로필은 실수
Json profile_value(double value, bool integers) {
    return integers ? Json(static_cast<std::int64_t>(value)) : Json(value);
}

Json profile_json(const ParameterProfile& profile, bool integers) {
    Json out = Json::object();
    for (const auto& [name, value] : profile) {
        json_set(out, name, profile_value(value, integers));
    }
    return out;
}

bool all_floats_finite(const Json& data) {
    return std::all_of(data.begin(), data.end(), [](const Json& value) {
        return !value.is_number_float() || std::isfinite(value.get<double>());
    });
}

// diagnostics의 clean(): 유한하지 않은 실수는 None
Json clean(const Json& value) {  // NOLINT(misc-no-recursion) - 상태 JSON 깊이만큼만 내려간다
    if (value.is_number_float()) {
        return std::isfinite(value.get<double>()) ? value : Json();
    }
    if (value.is_object()) {
        Json out = Json::object();
        for (const auto& item : value.items()) {
            json_set(out, item.key(), clean(item.value()));
        }
        return out;
    }
    if (value.is_array()) {
        Json out = Json::array();
        for (const Json& item : value) {
            out.push_back(clean(item));
        }
        return out;
    }
    return value;
}

std::string random_hex_id() {
    static thread_local std::mt19937_64 engine{std::random_device{}()};
    std::uniform_int_distribution<std::uint64_t> pick;
    std::array<char, 33> text{};
    // Fixed format and exact integer argument types.
    std::snprintf(text.data(), text.size(),  // NOLINT(cppcoreguidelines-pro-type-vararg)
                  "%016llx%016llx", static_cast<unsigned long long>(pick(engine)),
                  static_cast<unsigned long long>(pick(engine)));
    return {text.data()};
}

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

Job copy_job(const Job& job) { return std::make_shared<Json>(*job); }

float float_of(double value) { return static_cast<float>(value); }

}  // namespace

std::optional<std::pair<int, int>> mode_numbers(const std::string& mode) {
    for (const auto& [name, numbers] : kModes) {
        if (mode == name) {
            return numbers;
        }
    }
    return std::nullopt;
}

FlightLinkOptions FlightLink::default_options(const std::string& expected_uid,
                                              const std::filesystem::path& event_metadata) {
    FlightLinkOptions options;
    options.expected_uid = expected_uid;
    options.clock = [] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    };
    options.event_metadata = event_metadata;
    const char* owner = std::getenv("ARGUS_COMMAND_OWNER_FILE");
    options.command_owner_path = owner != nullptr ? owner : "";
    const char* marker = std::getenv("ARGUS_PROFILE_REBOOT_FILE");
    options.profile_state_path = marker != nullptr ? marker : "";
    options.link_factory = open_link;
    options.new_id = random_hex_id;
    return options;
}

FlightLink::FlightLink(FlightLinkOptions options)
    : expected_uid_(lower(options.expected_uid)),
      limits_(options.limits),
      clock_(std::move(options.clock)),
      event_metadata_(std::move(options.event_metadata)),
      link_factory_(options.link_factory ? std::move(options.link_factory)
                                         : LinkFactory(open_link)),
      new_id_(options.new_id ? std::move(options.new_id) : random_hex_id),
      command_owner_(options.command_owner_path) {
    static const std::regex kUidPattern("[0-9a-f]{36}");
    if (!std::regex_match(expected_uid_, kUidPattern)) {
        throw ValueError("FC UID는 18바이트 hex여야 합니다");
    }
    for (const char* kind : {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"}) {
        state_.pose_history[kind] = {};
    }
    state_.profile_state_path = options.profile_state_path;
    state_.profile_reboot_required =
        !state_.profile_state_path.empty() && std::filesystem::exists(state_.profile_state_path);
    state_.fc_events = std::make_unique<EventLog>(clock_, event_metadata_);
    // 2026-11-18: 빠진 자기장 보고 구성원을 아직 재요청 중인가 (EventLog는 재연결 때 바뀐다)
    mag_interference_.set_pending_query(
        [this](int sequence) { return state_.fc_events->is_pending(sequence); });
}

FlightLink::~FlightLink() { stop_worker(); }

void FlightLink::stop_worker() {
    quit_ = true;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void FlightLink::start() {
    if (!thread_.joinable()) {
        thread_ = std::thread([this] { thread_main(); });
    }
}

void FlightLink::thread_main() {
    while (!quit_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (quit_) {
            break;
        }
        step();
    }
}

void FlightLink::record_event(const std::string& text) {
    state_.events.push_back({{"at", now()}, {"text", text}});
    while (state_.events.size() > kMaxEvents) {
        state_.events.pop_front();
    }
}

void FlightLink::keepalive(const Json& client, bool ground_station) {
    if (!client.is_string() || utf8_length(client.get<std::string>()) < 1 ||
        utf8_length(client.get<std::string>()) > 80) {
        throw ValueError("관제 세션 ID 오류");
    }
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    const std::string name = client.get<std::string>();
    if (state_.owner && *state_.owner != name && now() - state_.lease_at <= limits_.lease_ttl) {
        throw ValueError("다른 GUI가 제어권을 사용 중입니다");
    }
    state_.owner = name;
    state_.lease_at = now();
    if (ground_station) {
        state_.gcs_client = name;
        state_.gcs_at = now();
    }
}

void FlightLink::request_connect(const Json& address, const Json& baud, const Json& client) {
    if (!address.is_string() || utf8_length(address.get<std::string>()) > 240) {
        throw ValueError("연결 주소 오류");
    }
    const std::string text = address.get<std::string>();
    static const std::regex kSerial(
        R"(/dev/(?:serial[01]|tty(?:ACM|USB|AMA|S)[0-9]+|serial/by-id/[^/]+))");
    static const std::regex kUdp(R"(udp(?:in|out):[A-Za-z0-9_.-]+:[0-9]{1,5})");
    const bool serial_ok = std::regex_match(text, kSerial);
    const bool udp_ok = std::regex_match(text, kUdp);
    if (!serial_ok && !udp_ok) {
        throw ValueError("/dev 또는 COM 직렬 장치, udpin:/udpout:host:port를 입력하세요");
    }
    if (udp_ok) {
        const int port = std::stoi(text.substr(text.rfind(':') + 1));
        if (port < 1 || port > 65535) {
            throw ValueError("UDP 포트 범위 오류");
        }
    }
    static const std::set<std::int64_t> kBauds{57600, 115200, 230400, 460800, 921600, 2000000};
    if (!baud.is_number_integer() || kBauds.count(as_int(baud)) == 0) {
        throw ValueError("지원하지 않는 baud rate");
    }
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    if (state_.link && (!fresh("HEARTBEAT", 2.5) || ground() != true)) {
        throw ValueError("기존 연결의 신선한 지상·무장 해제를 확인한 뒤 연결을 교체하세요");
    }
    if ((armed() || state_.owned_flight) && (state_.link || text != state_.connection)) {
        throw ValueError("무장/비행 중에는 연결을 교체할 수 없습니다");
    }
    keepalive(client);
    state_.jobs.push_front(
        std::make_shared<Json>(Json{{"action", "connect"}, {"address", text}, {"baud", baud}}));
}

Json FlightLink::submit(const Json& action_json, const Json& client, const Json& height_json) {
    static const std::set<std::string> kAllowed{
        "arm",      "disarm",         "takeoff",         "hold",           "land",      "set_home",
        "offboard", "indoor_profile", "outdoor_profile", "height_profile", "reboot",    "manual",
        "position", "altitude",       "stabilized",      "disconnect",     "set_height"};
    if (!action_json.is_string() || kAllowed.count(action_json.get<std::string>()) == 0) {
        throw ValueError("지원하지 않는 명령");
    }
    const std::string action = action_json.get<std::string>();
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    if (action != "land") {
        keepalive(client);
    }
    Json height = height_json;
    if (action == "takeoff" || action == "set_height") {
        height = validate_height(height_json);
    }
    if (action != "land" && (state_.pending || !state_.jobs.empty())) {
        throw ValueError("다른 명령 처리 중입니다. 긴급 착륙은 가능합니다");
    }
    if (action == "disconnect") {
        if (armed() || state_.owned_flight) {
            throw ValueError("무장/비행 중 연결 종료 불가");
        }
    } else {
        const std::string why = reason(action);
        if (!why.empty()) {
            throw ValueError(why);
        }
    }
    const Job job = std::make_shared<Json>(Json{{"id", new_id()},
                                                {"action", action},
                                                {"height", height},
                                                {"status", "queued"},
                                                {"message", "요청 대기"}});
    if (action == "land") {
        cancel("긴급 착륙 요청으로 취소");
        state_.jobs.clear();
        state_.jobs.push_front(job);
        state_.landing = true;
    } else {
        state_.jobs.push_back(job);
    }
    state_.last_job = job;
    return *job;
}

void FlightLink::set_height_band_source(HeightBandSource source) {
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    height_band_source_ = std::move(source);
}

std::optional<FlightLink::HeightBand> FlightLink::height_band() const {
    return height_band_source_ ? height_band_source_() : std::nullopt;
}

void FlightLink::set_demo_follow_source(DemoFollowSource source) {
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    demo_follow_source_ = std::move(source);
}

std::optional<bool> FlightLink::demo_follow() const {
    return demo_follow_source_ ? demo_follow_source_() : std::nullopt;
}

double FlightLink::validate_height(const Json& height_json) const {
    const std::optional<double> height = python_number(height_json);
    // 2026-11-18 데모 모드: 프로필 범위를 먼저 본다. 기존 한계, 센서 범위 검사는 그대로 이어진다.
    if (height) {
        if (const std::optional<HeightBand> band = height_band();
            band && (*height < band->minimum || *height > band->maximum)) {
            throw ValueError(band->message);
        }
    }
    if (!height || *height < limits_.minimum || *height > limits_.takeoff_maximum) {
        throw ValueError("목표 높이는 " + python_format_fixed(limits_.minimum, 1) + "–" +
                         python_format_fixed(limits_.takeoff_maximum, 1) + "m 범위입니다");
    }
    const Json* laser = sample("DISTANCE_SENSOR");
    const std::optional<double> laser_maximum =
        laser != nullptr ? python_number(get_or_null(laser, "max_distance"), .01) : std::nullopt;
    const auto flow_found = state_.params.find("SENS_FLOW_MAXHGT");
    const std::optional<double> flow_maximum =
        flow_found == state_.params.end() ? std::nullopt : python_number(flow_found->second);
    if (!laser_maximum || *height > *laser_maximum) {
        throw ValueError("목표 높이가 확인된 레이저 범위를 벗어납니다");
    }
    if (environment() != std::optional<std::string>("outdoor") &&
        (!laser_maximum || !flow_maximum || *height > std::min(*flow_maximum, *laser_maximum))) {
        throw ValueError("목표 높이가 확인된 레이저/옵티컬플로우 범위를 벗어납니다");
    }
    return *height;
}

const Json* FlightLink::sample(const std::string& kind) const {
    const auto found = state_.samples.find(kind);
    return found == state_.samples.end() ? nullptr : &found->second.data;
}

double FlightLink::age(const std::string& kind) const {
    const auto found = state_.samples.find(kind);
    return found == state_.samples.end() ? kInfinity : now() - found->second.at;
}

bool FlightLink::fresh(const std::string& kind, double ttl) const { return age(kind) <= ttl; }

bool FlightLink::armed() const {
    const Json* heartbeat = sample("HEARTBEAT");
    return heartbeat != nullptr && (as_int(at_key(*heartbeat, "base_mode")) & kSafetyArmed) != 0;
}

std::optional<std::string> FlightLink::mode() const {
    const Json* heartbeat = sample("HEARTBEAT");
    if (heartbeat == nullptr) {
        return std::nullopt;
    }
    const std::int64_t custom = as_int(at_key(*heartbeat, "custom_mode"));
    const int main_mode = static_cast<int>((custom >> 16) & 255);
    const int sub_mode = static_cast<int>((custom >> 24) & 255);
    for (const auto& [name, numbers] : kModes) {
        if (numbers.first == main_mode && numbers.second == sub_mode) {
            return std::string(name);
        }
    }
    return std::string("other");
}

std::optional<bool> FlightLink::ground() const {
    const Json* state = sample("EXTENDED_SYS_STATE");
    if (state == nullptr || !fresh("EXTENDED_SYS_STATE", 2.0)) {
        return std::nullopt;
    }
    const std::int64_t landed = as_int(at_key(*state, "landed_state"));
    if (landed == kLandedOnGround) {
        return true;
    }
    if (landed == kLandedInAir || landed == kLandedTakeoff || landed == kLandedLanding) {
        return false;
    }
    return std::nullopt;
}

bool FlightLink::imu_ok() const {
    const Json* state = sample("SYS_STATUS");
    return state != nullptr && fresh("SYS_STATUS", 2.0) &&
           (as_int(at_key(*state, "onboard_control_sensors_present")) & 3) == 3 &&
           (as_int(at_key(*state, "onboard_control_sensors_enabled")) & 3) == 3 &&
           (as_int(at_key(*state, "onboard_control_sensors_health")) & 3) == 3;
}

std::optional<bool> FlightLink::prearm_ok() const {
    const Json* state = sample("SYS_STATUS");
    if (state == nullptr || !fresh("SYS_STATUS", 2.0) ||
        (as_int(at_key(*state, "onboard_control_sensors_present")) & kPrearmCheck) == 0) {
        return std::nullopt;
    }
    return (as_int(at_key(*state, "onboard_control_sensors_health")) & kPrearmCheck) != 0;
}

bool FlightLink::local_ok() const {
    const Json* state = sample("ESTIMATOR_STATUS");
    if (state == nullptr || !fresh("ESTIMATOR_STATUS", 1.0) || !fresh("LOCAL_POSITION_NED", 0.5)) {
        return false;
    }
    const std::int64_t flags = as_int(at_key(*state, "flags"));
    const Json* position = sample("LOCAL_POSITION_NED");
    if ((flags & 15) != 15 || (flags & (32 | 64)) == 0) {
        return false;
    }
    if (!position_accuracy_reason().empty()) {
        return false;
    }
    static constexpr std::array<const char*, 6> kKeys{"x", "y", "z", "vx", "vy", "vz"};
    return std::all_of(kKeys.begin(), kKeys.end(), [&](const char* key) {
        return python_number(get_or_null(position, key)).has_value();
    });
}

std::string FlightLink::position_accuracy_reason() const {
    const Json* estimate = sample("ESTIMATOR_STATUS");
    const auto limit_found = state_.params.find("COM_POS_FS_EPH");
    const std::optional<double> limit =
        limit_found == state_.params.end() ? std::nullopt : python_number(limit_found->second);
    const std::optional<double> accuracy =
        estimate != nullptr ? python_number(get_or_null(estimate, "pos_horiz_accuracy"))
                            : std::nullopt;
    if (!fresh("ESTIMATOR_STATUS", 1.0) || !accuracy.has_value() || !limit.has_value()) {
        return "FC 수평 위치 정확도/허용 기준 수신을 기다리세요";
    }
    const double accuracy_m = accuracy.value_or(-1.0);
    const double limit_m = limit.value_or(-1.0);
    if (accuracy_m < 0) {
        return "FC 수평 위치 정확도/허용 기준 수신을 기다리세요";
    }
    // FC 기준값은 바꾸지 않는다. FC 히스테리시스가 잠시 더 허용해도 앱은 획득 기준을 요구한다.
    if (limit_m >= 0 && accuracy_m > limit_m) {
        return "FC 수평 위치 오차 " + python_format_fixed(accuracy_m, 2) + "m > 기준 " +
               python_format_fixed(limit_m, 2) + "m — 이륙 위치에서 추정을 다시 초기화하세요";
    }
    return "";
}

bool FlightLink::global_ok() const {
    const Json* state = sample("ESTIMATOR_STATUS");
    return state != nullptr && fresh("ESTIMATOR_STATUS", 1.0) &&
           (as_int(at_key(*state, "flags")) & 16) != 0 && fresh("GLOBAL_POSITION_INT", 1.0);
}

void FlightLink::store_profile_reboot_requirement() {
    if (state_.profile_state_path.empty()) {
        return;
    }
    const std::filesystem::path path(state_.profile_state_path);
    if (state_.profile_reboot_required) {
        std::filesystem::create_directories(path.parent_path());
        std::filesystem::path temporary = path;
        temporary.replace_extension(".next");
        {
            std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
            file << python_json_dumps_sorted(
                Json{{"expected_uid", expected_uid_}, {"reboot_required", true}});
            if (!file) {
                throw std::runtime_error("재부팅 필요 표시 쓰기 실패");
            }
        }
        std::filesystem::rename(temporary, path);
    } else {
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }
}

bool FlightLink::matches_profile(const ParameterProfile& profile) const {
    return std::all_of(profile.begin(), profile.end(), [this](const auto& item) {
        const auto found = state_.params.find(item.first);
        return found != state_.params.end() && python_equal(found->second, item.second);
    });
}

std::optional<std::string> FlightLink::environment() const {
    if (matches_profile(outdoor_profile())) {
        return std::string("outdoor");
    }
    if (matches_profile(indoor_profile())) {
        return std::string("indoor");
    }
    return std::nullopt;
}

bool FlightLink::baro_ok() const {
    const Json* state = sample("SYS_STATUS");
    return state != nullptr && fresh("SYS_STATUS", 2.0) &&
           (as_int(at_key(*state, "onboard_control_sensors_present")) & kAbsolutePressure) != 0 &&
           (as_int(at_key(*state, "onboard_control_sensors_enabled")) & kAbsolutePressure) != 0 &&
           (as_int(at_key(*state, "onboard_control_sensors_health")) & kAbsolutePressure) != 0;
}

std::string FlightLink::outdoor_navigation_reason() const {
    const Json* gps = sample("GPS_RAW_INT");
    const Json fix = get_or_null(gps, "fix_type");
    if (gps == nullptr || !fresh("GPS_RAW_INT", 3.0) || (fix.is_null() ? 0 : as_int(fix)) < 3 ||
        !global_ok()) {
        return "야외 GPS 3D fix·전역 위치 융합 미확인/만료";
    }
    return "";
}

std::string FlightLink::navigation_reason() const {
    if (environment() == std::optional<std::string>("outdoor")) {
        return outdoor_navigation_reason();
    }
    return flow_ok() ? "" : "신선한 플로우 입력이 필요합니다";
}

std::optional<Json> FlightLink::home_signature() const {
    const Json* home = sample("HOME_POSITION");
    if (home == nullptr) {
        return std::nullopt;
    }
    Json values = Json::array();
    for (const char* key : {"latitude", "longitude", "altitude"}) {
        values.push_back(get_or_null(home, key));
    }
    for (const Json& value : values) {
        if (!python_number(value)) {
            return std::nullopt;
        }
    }
    return values;
}

std::string FlightLink::height_sample_kind() const {
    return environment() == std::optional<std::string>("outdoor") ? "GLOBAL_POSITION_INT"
                                                                  : "DISTANCE_SENSOR";
}

std::pair<std::optional<double>, std::string> FlightLink::height() const {
    // 두 프로필 모두 지면 기준 거리 높이를 쓴다. 이륙 때 HOME_POSITION이 바뀌어도
    // 고도 목표를 바꾸거나 상승을 취소하지 않는다.
    const Json* laser = sample("DISTANCE_SENSOR");
    if (laser == nullptr || !fresh("DISTANCE_SENSOR", limits_.sensor_ttl)) {
        return {std::nullopt, "레이저 만료/미수신"};
    }
    const std::optional<double> value = python_number(at_key(*laser, "current_distance"), 0.01);
    const std::optional<double> low = python_number(at_key(*laser, "min_distance"), 0.01);
    const std::optional<double> high = python_number(at_key(*laser, "max_distance"), 0.01);
    if (!value || !low || !high || *low <= 0 || *low > *value || *value > *high) {
        return {std::nullopt, "레이저 유효 범위 밖"};
    }
    const Json* altitude = sample("ALTITUDE");
    const std::optional<double> terrain =
        altitude != nullptr ? python_number(get_or_null(altitude, "altitude_terrain"))
                            : std::nullopt;
    const Json* estimate = sample("ESTIMATOR_STATUS");
    if (terrain && *terrain >= 0 && fresh("ALTITUDE", 0.3) && estimate != nullptr &&
        fresh("ESTIMATOR_STATUS", 1.0) && (as_int(at_key(*estimate, "flags")) & 64) != 0) {
        return {terrain, "FC 지면 높이 · 레이저 유효"};
    }
    return {value, "레이저 거리 · 기울기 보정 전"};
}

bool FlightLink::flow_ok() const {
    const Json* flow = sample("OPTICAL_FLOW_RAD");
    const auto found = state_.params.find("EKF2_OF_QMIN");
    const std::optional<double> minimum =
        found == state_.params.end() ? std::nullopt : python_number(found->second);
    return flow != nullptr && minimum && fresh("OPTICAL_FLOW_RAD", 0.5) &&
           as_double(at_key(*flow, "quality")) > std::max(0.0, *minimum);
}

bool FlightLink::profile_ok() const {
    return !state_.profile_reboot_required && environment().has_value();
}

std::string FlightLink::reason(const std::string& action) const {
    if (!state_.link || state_.uid != expected_uid_) {
        return "FC 미연결 또는 UID 확인 미완료/불일치";
    }
    if (action == "land") {
        return "";
    }
    if (!fresh("HEARTBEAT", limits_.heartbeat_ttl)) {
        return "FC heartbeat 만료";
    }
    if (state_.landing) {
        return "착륙 진행 중 — 실제 지상/무장 해제 확인 대기";
    }
    if (action == "disarm") {
        return ground() == true ? "" : "지상 상태를 확인한 뒤 Disarm할 수 있습니다";
    }
    if (action == "height_profile") {
        if (armed() || ground() != true) {
            return "고도 제한 설정은 신선한 지상·무장 해제에서만 가능합니다";
        }
        for (const auto& item : height_profile()) {
            const auto type = state_.param_types.find(item.first);
            if (state_.params.count(item.first) == 0 || type == state_.param_types.end() ||
                type->second != kParamReal32) {
                return "FC 플로우 고도 파라미터 값/실수 타입 수신을 기다리세요";
            }
        }
        return "";
    }
    if (action == "indoor_profile" || action == "outdoor_profile" || action == "reboot") {
        if (armed() || ground() != true) {
            const std::string label = action == "reboot" ? "FC 재부팅" : "센서 모드 설정";
            return label + "은 신선한 지상·무장 해제에서만 가능합니다";
        }
        if (action == "reboot") {
            return "";
        }
        const ParameterProfile& profile =
            action == "outdoor_profile" ? outdoor_profile() : indoor_profile();
        for (const auto& item : profile) {
            if (state_.param_types.count(item.first) == 0 || state_.params.count(item.first) == 0) {
                return "FC 센서 파라미터의 값/타입 수신을 기다리세요";
            }
        }
        for (const auto& item : profile) {
            const int type = state_.param_types.at(item.first);
            if (type != kParamInt32 && type != kParamUint32) {
                return "FC 센서 파라미터 타입이 예상과 다릅니다";
            }
        }
        return "";
    }
    if (action == "position" || action == "altitude" || action == "stabilized" ||
        action == "manual" || action == "hold") {
        return "";
    }
    if (action == "set_home") {
        if (armed() || ground() != true) {
            return "SetHome은 지상·무장 해제에서만 가능합니다";
        }
        return global_ok()
                   ? ""
                   : "FC의 유효한 전역 위치가 필요합니다 — 레이저 높이 zero 기능이 아닙니다";
    }
    if (!imu_ok()) {
        return "FC gyro/acc 상태 미확인/비정상";
    }
    const std::optional<std::string> where = environment();
    if (action == "arm") {
        if (state_.profile_reboot_required) {
            return "센서 설정 변경 후 FC 재부팅이 필요합니다";
        }
        if (where == std::optional<std::string>("outdoor") &&
            !outdoor_navigation_reason().empty()) {
            return outdoor_navigation_reason();
        }
        if (armed()) {
            return "이미 무장 상태입니다";
        }
        if (std::string mag = mag_arm_reason(); !mag.empty()) {
            return mag;
        }
        const std::optional<std::string> current = mode();
        if (current == std::optional<std::string>("takeoff") ||
            current == std::optional<std::string>("offboard")) {
            return "먼저 Position 또는 Altitude 모드로 전환한 뒤 ARM하세요";
        }
        if (prearm_ok() == false) {
            return "FC 사전 검사 실패 — FC 이벤트/상태를 확인하세요";
        }
        return ground() == true ? "" : "신선한 지상 상태가 필요합니다";
    }
    std::string accuracy = position_accuracy_reason();
    if (!accuracy.empty()) {
        return accuracy;
    }
    if (!local_ok()) {
        return "FC 로컬 위치·속도·높이 추정 미확인/무효";
    }
    if (!height().first || !navigation_reason().empty()) {
        return where == std::optional<std::string>("outdoor")
                   ? "야외 GPS·레이저 입력 확인 필요"
                   : "신선한 레이저·플로우 입력이 필요합니다";
    }
    if (!where) {
        return "실내/야외 센서 설정 확인이 필요합니다";
    }
    if (!armed()) {
        return "먼저 별도 Arm 버튼으로 무장하세요";
    }
    if (action == "takeoff") {
        if (std::string mag = mag_takeoff_reason(); !mag.empty()) {
            return mag;
        }
    }
    if (action == "set_height") {
        if (!state_.external || !state_.owned_flight || !state_.owned_mode_confirmed ||
            state_.holding || !state_.local_target ||
            mode() != std::optional<std::string>("offboard") || ground() != false) {
            return "이 GUI의 이륙 완료 후 Offboard 위치 유지 중에만 고도를 변경할 수 있습니다";
        }
    }
    if (action == "takeoff" && ground() != true) {
        return "Takeoff는 지상에서만 요청할 수 있습니다";
    }
    if (action == "takeoff" &&
        (!fresh("ATTITUDE", .5) || !python_number(get_or_null(sample("ATTITUDE"), "yaw")))) {
        return "신선한 FC 방향 추정이 필요합니다";
    }
    if (action == "offboard" && ground() != false) {
        return "외부 hover는 이륙/비행 상태에서 시작합니다";
    }
    return "";
}

std::string FlightLink::mag_arm_reason() const {
    return mag_interference_.active() == std::optional<bool>(true) ? kMagArmRefusal : "";
}

std::string FlightLink::mag_takeoff_reason() const {
    return mag_interference_.armed_with_warning() ? kMagTakeoffRefusal : "";
}

std::optional<bool> FlightLink::known_armed() const {
    if (sample("HEARTBEAT") == nullptr || !fresh("HEARTBEAT", limits_.heartbeat_ttl)) {
        return std::nullopt;
    }
    return armed();
}

void FlightLink::record_mag_change(MagInterferenceMonitor::Change change) {
    if (change == MagInterferenceMonitor::Change::kSet) {
        record_event("FC 강한 자기장 간섭 경고 감지");
    } else if (change == MagInterferenceMonitor::Change::kCleared) {
        record_event("FC 강한 자기장 간섭 경고 해제");
    } else if (change == MagInterferenceMonitor::Change::kUnknown) {
        record_event("FC 자기장 경고 보고 불완전 - 상태 미확인, 재연결 권장");
    }
}

void FlightLink::update_mag_interference() {
    record_mag_change(mag_interference_.update(state_.fc_events->epoch(), known_armed(),
                                               state_.fc_events->latest(), now()));
}

bool FlightLink::gcs_active() const {
    return state_.owner == state_.gcs_client && state_.gcs_client.has_value() &&
           now() - state_.gcs_at <= limits_.lease_ttl;
}

bool FlightLink::follow_busy() const {
    const auto& follow = state_.follow_b;
    return follow &&
           (follow->token_active() || follow->bench_active() || follow->manual_yaw().active());
}

Json FlightLink::hold_view() {
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    // 아래 식은 snapshot()의 같은 키와 같다 (GatewayStage2Test가 비교한다)
    const bool fresh_heartbeat = fresh("HEARTBEAT", limits_.heartbeat_ttl);
    const auto height_value = height().first;
    const double height_age = age(height_sample_kind());
    const std::optional<std::string> where = environment();
    return Json{{"fresh", fresh_heartbeat},
                {"identity_ok", state_.uid == expected_uid_},
                {"mode", mode() ? Json(*mode()) : Json()},
                {"armed", fresh_heartbeat ? Json(armed()) : Json()},
                {"ground", ground() ? Json(*ground()) : Json()},
                {"local_ok", fresh("ESTIMATOR_STATUS", 1) && fresh("LOCAL_POSITION_NED", .5)
                                 ? Json(local_ok())
                                 : Json()},
                {"height_m", optional_number(height_value)},
                {"height_age_ms", std::isfinite(height_age) ? Json(height_age * 1000) : Json()},
                {"flight_environment", where ? Json(*where) : Json()}};
}

Json FlightLink::snapshot() {
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    const Json* laser = sample("DISTANCE_SENSOR");
    const Json* flow = sample("OPTICAL_FLOW_RAD");
    const Json* altitude = sample("ALTITUDE");
    const Json* attitude = sample("ATTITUDE");
    const Json* gps = sample("GPS_RAW_INT");
    const Json* rc = sample("RC_CHANNELS");
    const Json* sys_status = sample("SYS_STATUS");
    const Json* battery = sample("BATTERY_STATUS");
    Json volts;
    Json amps;
    Json remaining;
    if (battery != nullptr && fresh("BATTERY_STATUS", 3.0)) {
        std::int64_t total = 0;
        bool any = false;
        const Json voltages = at_key(*battery, "voltages");
        for (const Json& value : voltages) {
            const std::int64_t cell = as_int(value);
            if (cell > 0 && cell < 65535) {
                total += cell;
                any = true;
            }
        }
        volts = any ? Json(static_cast<double>(total) / 1000) : Json();
        const std::int64_t current = as_int(at_key(*battery, "current_battery"));
        amps = current != -1 ? Json(static_cast<double>(current) / 100) : Json();
        const std::int64_t left = as_int(at_key(*battery, "battery_remaining"));
        remaining = left >= 0 ? Json(left) : Json();
    } else if (sys_status != nullptr && fresh("SYS_STATUS", 2.0)) {
        const std::int64_t voltage = as_int(at_key(*sys_status, "voltage_battery"));
        volts = voltage > 0 && voltage < 65535 ? Json(static_cast<double>(voltage) / 1000) : Json();
        const std::int64_t current = as_int(at_key(*sys_status, "current_battery"));
        amps = current != -1 ? Json(static_cast<double>(current) / 100) : Json();
        const std::int64_t left = as_int(at_key(*sys_status, "battery_remaining"));
        remaining = left >= 0 ? Json(left) : Json();
    }
    const auto [height_value, height_source] = height();
    Json reasons = Json::object();
    for (const char* action : kSnapshotActions) {
        if (std::string(action) == "disconnect") {
            json_set(reasons, action,
                     armed() || state_.owned_flight ? "무장/비행 중 연결 종료 불가" : "");
        } else {
            json_set(reasons, action, reason(action));
        }
    }
    const bool busy = state_.pending || !state_.jobs.empty() ||
                      (state_.follow_b && state_.follow_b->manual_yaw().active());
    Json controls = Json::object();
    for (const char* action : kSnapshotActions) {
        json_set(controls, action,
                 reasons.at(action).get<std::string>().empty() &&
                     (std::string(action) == "land" || !busy));
    }
    const bool fresh_heartbeat = fresh("HEARTBEAT", limits_.heartbeat_ttl);
    const double laser_age = age("DISTANCE_SENSOR");
    const Json* home_sample = sample("HOME_POSITION");
    Json home;
    if (home_sample != nullptr) {
        home = {
            {"latitude", optional_number(python_number(at_key(*home_sample, "latitude"), 1e-7))},
            {"longitude", optional_number(python_number(at_key(*home_sample, "longitude"), 1e-7))},
            {"altitude_m", optional_number(python_number(at_key(*home_sample, "altitude"), .001))},
            {"global_valid", global_ok()}};
    }
    const Json* estimate = sample("ESTIMATOR_STATUS");
    Json attitude_deg;
    if (attitude != nullptr && fresh("ATTITUDE", .5)) {
        attitude_deg = Json::array();
        for (const char* key : {"roll", "pitch", "yaw"}) {
            attitude_deg.push_back(
                optional_number(python_number(get_or_null(attitude, key), 180 / M_PI)));
        }
    }
    Json gps_json;
    if (gps != nullptr && fresh("GPS_RAW_INT", 3.0)) {
        const Json satellites = at_key(*gps, "satellites_visible");
        gps_json = {{"fix", at_key(*gps, "fix_type")},
                    {"satellites", as_int(satellites) != 255 ? satellites : Json()}};
    }
    Json rc_input;
    Json rc_throttle;
    if (rc != nullptr && fresh("RC_CHANNELS", 1.0)) {
        rc_input = as_int(at_key(*rc, "chancount")) > 0;
        const auto found = state_.params.find("RC_MAP_THROTTLE");
        const Json mapped = found == state_.params.end() ? Json(0) : found->second;
        const double channel = as_double(mapped);
        if (channel >= 1 && channel <= 18) {
            rc_throttle = get_or_null(rc, "chan" + std::to_string(as_int(mapped)) + "_raw");
        }
    }
    Json params = Json::object();
    for (const auto& [name, value] : state_.params) {
        json_set(params, name, value);
    }
    const std::optional<std::string> where = environment();
    const std::string height_kind = height_sample_kind();
    const double height_age = age(height_kind);
    Json events = Json::array();
    const std::size_t first = state_.events.size() > 12 ? state_.events.size() - 12 : 0;
    for (std::size_t i = first; i < state_.events.size(); ++i) {
        events.push_back(state_.events.at(i));
    }
    const auto accuracy_limit = state_.params.find("COM_POS_FS_EPH");
    Json out = {
        {"connection", state_.connection},
        {"linked", static_cast<bool>(state_.link)},
        {"fresh", fresh_heartbeat},
        {"manual_yaw",
         state_.follow_b ? state_.follow_b->manual_yaw().status() : Json{{"supported", false}}},
        {"uid", state_.uid ? Json(*state_.uid) : Json()},
        {"expected_uid", expected_uid_},
        {"identity_ok", state_.uid == expected_uid_},
        {"gui_heartbeat_active", gcs_active()},
        {"gui_gcs_identity", "244/190"},
        {"firmware", state_.firmware ? Json(*state_.firmware) : Json()},
        {"mode", mode() ? Json(*mode()) : Json()},
        {"armed", fresh_heartbeat ? Json(armed()) : Json()},
        {"ground", ground() ? Json(*ground()) : Json()},
        {"imu_ok", fresh("SYS_STATUS", 2) ? Json(imu_ok()) : Json()},
        {"prearm_ok", prearm_ok() ? Json(*prearm_ok()) : Json()},
        {"local_ok", fresh("ESTIMATOR_STATUS", 1) && fresh("LOCAL_POSITION_NED", .5)
                         ? Json(local_ok())
                         : Json()},
        {"height_m", optional_number(height_value)},
        {"height_source", height_source},
        {"laser_age_ms", std::isfinite(laser_age) ? Json(laser_age * 1000) : Json()},
        {"raw_laser_m",
         laser != nullptr && fresh("DISTANCE_SENSOR", .25)
             ? optional_number(python_number(at_key(*laser, "current_distance"), .01))
             : Json()},
        {"relative_altitude_m",
         altitude != nullptr && fresh("ALTITUDE", .5)
             ? optional_number(python_number(get_or_null(altitude, "altitude_relative")))
             : Json()},
        {"position_accuracy_m",
         estimate != nullptr && fresh("ESTIMATOR_STATUS", 1.0)
             ? optional_number(python_number(get_or_null(estimate, "pos_horiz_accuracy")))
             : Json()},
        {"position_accuracy_limit_m", accuracy_limit == state_.params.end()
                                          ? Json()
                                          : optional_number(python_number(accuracy_limit->second))},
        {"flow_quality",
         flow != nullptr && fresh("OPTICAL_FLOW_RAD", .5) ? at_key(*flow, "quality") : Json()},
        {"attitude_deg", attitude_deg},
        {"gps", gps_json},
        {"battery", {{"voltage", volts}, {"current", amps}, {"remaining", remaining}}},
        {"rc_input", rc_input},
        {"rc_throttle_us", rc_throttle},
        {"home", home},
        {"params", params},
        {"busy", busy},
        {"height_profile",
         {{"ready", matches_profile(height_profile())},
          {"expected", profile_json(height_profile(), false)}}},
        {"flight_environment", where ? Json(*where) : Json()},
        {"cmc_streams", cmc_stream_status()},
        {"outdoor_profile",
         {{"ready", matches_profile(outdoor_profile()) && !state_.profile_reboot_required},
          {"expected", profile_json(outdoor_profile(), true)},
          {"reboot_required", state_.profile_reboot_required}}},
        {"baro_ok", baro_ok()},
        {"height_reference", "ground"},
        {"height_age_ms", std::isfinite(height_age) ? Json(height_age * 1000) : Json()},
        {"indoor_profile",
         {{"ready", matches_profile(indoor_profile()) && !state_.profile_reboot_required},
          {"reboot_required", state_.profile_reboot_required},
          {"expected", profile_json(indoor_profile(), true)}}},
        {"takeoff_path", where == std::optional<std::string>("outdoor")
                             ? "Offboard / LOCAL_NED / GPS + range"
                             : "Offboard / LOCAL_NED / optical flow + range"},
        {"local_target", state_.local_target ? *state_.local_target : Json()},
        {"external", state_.external},
        {"owned_flight", state_.owned_flight},
        {"landing", state_.landing},
        {"controls", controls},
        {"reasons", reasons},
        {"last_job", state_.last_job ? *state_.last_job : Json()},
        {"events", events},
        {"fc_events", state_.fc_events->snapshot()},
        {"mag_interference", mag_interference_.status()},
        {"minimum_m", limits_.minimum},
        {"ceiling_m", limits_.ceiling},
        {"takeoff_maximum_m", limits_.takeoff_maximum},
        {"height_change_maximum_m", limits_.takeoff_maximum},
        {"brake_height_m", limits_.brake_height()},
        {"takeoff_timeout_s", limits_.takeoff_timeout},
    };
    return out;
}

Json FlightLink::diagnostics() {
    // 조회는 제어권/lease를 갱신하거나 FC 명령을 보내지 않는다.
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    Json samples = Json::object();
    for (const char* kind :
         {"HEARTBEAT", "SYSTEM_TIME", "ATTITUDE", "EXTENDED_SYS_STATE", "ESTIMATOR_STATUS",
          "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "ALTITUDE", "DISTANCE_SENSOR",
          "OPTICAL_FLOW_RAD", "POSITION_TARGET_LOCAL_NED", "ATTITUDE_TARGET", "SERVO_OUTPUT_RAW",
          "RC_CHANNELS"}) {
        if (state_.samples.count(kind) != 0) {
            json_set(samples, kind, {{"age_s", age(kind)}, {"data", *sample(kind)}});
        }
    }
    Json types = Json::object();
    for (const auto& [name, type] : state_.param_types) {
        json_set(types, name, type);
    }
    return clean(Json{{"state", snapshot()}, {"samples", samples}, {"param_types", types}});
}

Json FlightLink::observations() {
    // 원래 호스트 수신 시각이다. 노출/FC 시계 인증이 아니다.
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    Json observation = diagnostics();
    Json history = Json::object();
    for (const auto& [kind, rows] : state_.pose_history) {
        Json list = Json::array();
        for (const Json& row : rows) {
            list.push_back(row);
        }
        json_set(history, kind, std::move(list));
    }
    json_set(observation, "schema", "argus.fc.observations.v1");
    json_set(observation, "clock_domain", "host_monotonic");
    json_set(observation, "timestamp_basis", "host_receive");
    json_set(observation, "epoch", state_.pose_epoch);
    json_set(observation, "history", history);
    json_set(observation, "sensors", {{"flow_valid", flow_ok()}, {"local_valid", local_ok()}});
    json_set(observation, "command_owner", command_owner_.status());
    return observation;
}

Json FlightLink::yaw_handoff() {
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    auto& follow = state_.follow_b;
    if (follow && follow->manual_yaw().active()) {
        throw ValueError("수동 yaw 시험을 먼저 중지하세요");
    }
    if (state_.uid != expected_uid_ || !fresh("HEARTBEAT", 1.5)) {
        throw ValueError("FC UID/heartbeat 미확인");
    }
    if (state_.pending || !state_.jobs.empty() || state_.landing) {
        throw ValueError("FC 명령 처리 중");
    }
    if (!armed() || ground() != false) {
        throw ValueError("yaw 인계는 이미 공중인 기체의 명시적 START에만 허용됩니다");
    }
    const std::optional<std::string> current = mode();
    if (current == std::optional<std::string>("hold") ||
        current == std::optional<std::string>("position")) {
        if (follow) {
            follow->set_rc_override(false);
        }
        state_.external = state_.owned_flight = state_.owned_mode_confirmed = false;
        state_.local_target.reset();
        command_owner_.release();
        if (!command_owner_.status().at("available").get<bool>()) {
            throw ValueError("다른 setpoint 송신자가 제어권을 사용 중");
        }
        return {{"accepted", true}, {"ready", true}, {"mode", *current}};
    }
    if (current != std::optional<std::string>("offboard") || !command_owner_.held()) {
        throw ValueError("앱 소유 Offboard 또는 실제 Hold가 필요합니다");
    }
    if (!height().first || !local_ok() || !navigation_reason().empty()) {
        throw ValueError("인계 전 센서/위치 추정이 유효해야 합니다");
    }
    if (follow && follow->legacy_mode_handoff()) {
        // Native ICD 정비 경로는 원격 소유자와 인계 계약이 따로 있다.
        const Job job = std::make_shared<Json>(Json{{"id", new_id()},
                                                    {"action", "yaw_handoff"},
                                                    {"status", "queued"},
                                                    {"message", "실제 Hold 인계 대기"}});
        state_.jobs.push_back(job);
        state_.last_job = job;
        return {{"accepted", true}, {"ready", false}, {"job_id", (*job).at("id")}};
    }
    if (!follow) {
        throw ValueError("추종 송신부가 없습니다");
    }
    follow->prepare_handoff();
    return {{"accepted", true},
            {"ready", true},
            {"mode", "offboard"},
            {"reason", "기존 고도 유지 중 · 첫 유효 추종 명령에서 인계"}};
}

void FlightLink::write(const Bytes& bytes) const {
    if (!state_.link) {
        throw std::runtime_error("'NoneType' object has no attribute 'mav'");
    }
    state_.link->write(bytes);
}

void FlightLink::send_command(std::uint16_t command, const std::vector<double>& args) {
    std::array<float, 7> values{};
    for (std::size_t i = 0; i < args.size() && i < values.size(); ++i) {
        values.at(i) = float_of(args.at(i));
    }
    if (!state_.mav) {
        throw std::runtime_error("'NoneType' object has no attribute 'mav'");
    }
    write(state_.mav->command_long(1, 1, command, 0, values));
}

void FlightLink::send_position_target(std::uint8_t frame, std::uint16_t mask,
                                      const std::array<float, 11>& values) {
    if (!state_.mav) {
        throw std::runtime_error("'NoneType' object has no attribute 'mav'");
    }
    write(state_.mav->set_position_target_local_ned(0, 1, 1, frame, mask, values));
}

void FlightLink::send_mode(const std::string& mode_name) {
    const auto numbers = mode_numbers(mode_name);
    if (!numbers) {
        key_error(mode_name);
    }
    send_command(kCmdDoSetMode,
                 {1, static_cast<double>(numbers->first), static_cast<double>(numbers->second)});
}

void FlightLink::zero() {
    // 위치/가속도/yaw는 무시하고 영속도·yaw rate 0을 보낸다. 오래된 관측으로 반복하지 않는다.
    if (!command_owner_.acquire()) {
        throw ValueError("다른 setpoint 송신자가 제어권을 사용 중");
    }
    write(state_.mav->set_position_target_local_ned(0, 1, 1, kFrameLocalNed, kZeroMask, {}));
    state_.last_tx = now();
}

void FlightLink::cancel(const std::string& message) {
    if (state_.pending) {
        json_set((*state_.pending), "status", "cancelled");
        json_set((*state_.pending), "message", message);
        record_event(message);
        state_.pending.reset();
    }
}

bool FlightLink::takeoff_climb_stalled() {
    if (!state_.pending || get_or_null(state_.pending.get(), "action") != Json("takeoff")) {
        return false;
    }
    Json& job = *state_.pending;
    const auto reset = [&job] {
        json_set(job, "stall_since", Json());
        return false;
    };
    if (!python_truthy(get_or_null(&job, "ack")) || !armed() || ground() != false ||
        mode() != std::optional<std::string>("offboard") || !state_.local_target) {
        return reset();
    }
    const Json& target = *state_.local_target;
    const Json climb_at = at_key(target, "climb_at");
    if (climb_at.is_null() ||
        now() - as_double(climb_at) <
            std::abs(as_double(at_key(target, "distance"))) / limits_.climb_speed) {
        return reset();
    }
    double floor = limits_.takeoff_stall_min_height;
    if (const auto band = height_band()) {
        floor = std::max(floor, band->minimum);
    }
    const std::optional<double> current = height().first;
    const Json* position = sample("LOCAL_POSITION_NED");
    if (!current || *current < floor || position == nullptr ||
        !fresh("LOCAL_POSITION_NED", limits_.sensor_ttl) ||
        !(std::abs(as_double(at_key(*position, "vz"))) < limits_.takeoff_stall_speed)) {
        return reset();
    }
    const Json since = get_or_null(&job, "stall_since");
    if (since.is_null()) {
        json_set(job, "stall_since", now());
        return false;
    }
    return now() - as_double(since) >= limits_.takeoff_stall_hold_s;
}

double FlightLink::ramped_z() const {
    if (!state_.local_target) {
        throw ValueError("고도 유지 목표가 없습니다");
    }
    const Json& target = *state_.local_target;
    const Json climb_at = at_key(target, "climb_at");
    const double elapsed = climb_at.is_null() ? 0.0 : std::max(0.0, now() - as_double(climb_at));
    const double distance = as_double(at_key(target, "distance"));
    const double dz =
        std::copysign(std::min(std::abs(distance), limits_.climb_speed * elapsed), distance);
    return as_double(at_key(target, "start_z")) - dz;
}

void FlightLink::local_setpoint() {
    if (!state_.local_target) {
        return;
    }
    const Json& target = *state_.local_target;
    if (!command_owner_.acquire()) {
        throw ValueError("다른 setpoint 송신자가 제어권을 사용 중");
    }
    // 모드 수락 전에는 start_z를 유지하고, 확인된 Offboard에서 올리거나 내린다.
    std::uint16_t mask = kPositionMask;
    const std::optional<double> yaw_rate =
        state_.follow_b ? state_.follow_b->manual_yaw().position_rate() : std::nullopt;
    if (yaw_rate) {
        mask = static_cast<std::uint16_t>((mask & ~kYawRateIgnore) | kYawIgnore);
    }
    write(state_.mav->set_position_target_local_ned(
        0, 1, 1, kFrameLocalNed, mask,
        {float_of(as_double(at_key(target, "x"))), float_of(as_double(at_key(target, "y"))),
         float_of(ramped_z()), 0, 0, 0, 0, 0, 0, float_of(as_double(at_key(target, "yaw"))),
         yaw_rate ? float_of(*yaw_rate) : 0.0F}));
    state_.last_tx = now();
}

void FlightLink::advance_profile(const Job& job) {
    Json& data = *job;
    const std::string action = at_key(data, "action").get<std::string>();
    if (at_key(data, "changes").empty()) {
        std::string message = "GPS 없는 옵티컬플로우·레이저 설정 확인됨";
        if (action == "height_profile") {
            message = "FC 플로우 고도 상한 5.0m 확인됨";
        } else if (action == "outdoor_profile") {
            message = "야외 GPS 위치·레이저 고도 설정 확인됨";
        }
        if (state_.profile_reboot_required) {
            message += " — FC 재부팅 후 다시 연결하세요";
        }
        finish("done", message);
        return;
    }
    if (state_.uid != expected_uid_ || !fresh("HEARTBEAT", limits_.heartbeat_ttl) || armed() ||
        ground() != true) {
        finish("failed", "센서 설정 중 지상·무장 해제 확인 유실 — 추가 쓰기 중단");
        return;
    }
    const Json change = at_key(data, "changes").at(0);
    const std::string name = change.at(0).get<std::string>();
    const Json& value = change.at(1);
    const auto type_found = state_.param_types.find(name);
    if (type_found == state_.param_types.end()) {
        key_error(name);
    }
    const int kind = type_found->second;
    const bool allowed = action == "height_profile" ? kind == kParamReal32
                                                    : (kind == kParamInt32 || kind == kParamUint32);
    if (!allowed) {
        finish("failed", "센서 파라미터 타입이 예상과 다릅니다: " + name);
        return;
    }
    float encoded = float_of(as_double(value));
    if (state_.param_bytewise && (kind == kParamInt32 || kind == kParamUint32)) {
        const std::int64_t integer = as_int(value);
        if (kind == kParamUint32 && integer < 0) {
            throw std::runtime_error("argument out of range");
        }
        const auto bits = static_cast<std::uint32_t>(integer);
        std::memcpy(&encoded, &bits, sizeof(encoded));
    }
    json_set(data, "waiting_param", name);
    json_set(data, "waiting_value", value);
    json_set(data, "waiting_type", kind);
    write(state_.mav->param_set(1, 1, name, encoded, static_cast<std::uint8_t>(kind)));
}

void FlightLink::finish(const std::string& status, std::string message) {
    if (state_.pending) {
        Json& pending = *state_.pending;
        if (status == "rejected" && python_truthy(get_or_null(&pending, "fc_reason"))) {
            message += " · " + pending.at("fc_reason").get<std::string>();
        }
        const std::string action = at_key(pending, "action").get<std::string>();
        if ((status == "rejected" || status == "failed") &&
            (action == "takeoff" || action == "offboard")) {
            state_.owned_flight = state_.external = false;
            state_.local_target.reset();
        }
        if ((status == "rejected" || status == "failed") && action == "land") {
            state_.landing = state_.landing_mode_seen = false;
        }
        json_set(pending, "status", status);
        json_set(pending, "message", message);
        state_.last_job = state_.pending;
    }
    record_event(message);
    state_.pending.reset();
}

void FlightLink::dispatch(const Job& job) {
    Json& data = *job;
    const std::string action = at_key(data, "action").get<std::string>();
    auto& follow = state_.follow_b;
    if (action == "hold" && follow && follow->offboard_hover_enabled() &&
        mode() == std::optional<std::string>("offboard")) {
        if (!follow->establish_hover("operator_hold", false)) {
            state_.last_job = copy_job(job);
            json_set((*state_.last_job), "status", "rejected");
            json_set((*state_.last_job), "message",
                     "Offboard 고도 유지 조건 미충족 · 기존 감독 유지");
            return;
        }
        follow->invalidate("released");
        state_.last_job = copy_job(job);
        json_set((*state_.last_job), "status", "done");
        json_set((*state_.last_job), "message", "Offboard 고도 유지 확인");
        return;
    }
    if (follow &&
        (follow->token_active() || follow->bench_active() || follow->manual_yaw().active())) {
        follow->clear_bench();
        follow->invalidate("operator_command");
    }
    if (action == "connect") {
        close_link();
        state_.samples.clear();
        state_.params.clear();
        state_.param_types.clear();
        state_.sensor_stamps.clear();
        state_.boot_stamps.clear();
        for (auto& [kind, rows] : state_.pose_history) {
            rows.clear();
        }
        state_.pose_epoch += 1;
        state_.fc_events = std::make_unique<EventLog>(clock_, event_metadata_);
        mag_interference_.reset(state_.fc_events->epoch());
        state_.uid.reset();
        state_.firmware.reset();
        state_.config_requested = false;
        state_.last_probe = -kInfinity;
        const std::string address = at_key(data, "address").get<std::string>();
        state_.link = link_factory_(address, static_cast<int>(as_int(at_key(data, "baud"))));
        state_.mav = std::make_unique<MavlinkSender>(245, 191);
        state_.parser = MavlinkParser();
        state_.gcs_mav = std::make_unique<MavlinkSender>(244, 190);
        state_.last_gcs_tx = -kInfinity;
        state_.connection = address;
        record_event("FC 링크 열림 — heartbeat/UID/실제 센서 확인 대기");
        return;
    }
    if (action == "disconnect") {
        close_link();
        state_.uid.reset();
        state_.last_job = copy_job(job);
        json_set((*state_.last_job), "status", "done");
        json_set((*state_.last_job), "message", "링크 종료");
        return;
    }
    std::string why = reason(action == "yaw_handoff" ? "hold" : action);
    if ((action == "takeoff" || action == "offboard" || action == "set_height") &&
        !(command_owner_.held() || command_owner_.status().at("available").get<bool>())) {
        why = "yaw setpoint 송신 중 — 먼저 yaw를 중지하세요";
    }
    if (why.empty() && (action == "takeoff" || action == "set_height")) {
        try {
            validate_height(at_key(data, "height"));
        } catch (const ValueError& error) {
            why = error.what();
        }
    }
    if (!why.empty()) {
        state_.last_job = copy_job(job);
        json_set((*state_.last_job), "status", "rejected");
        json_set((*state_.last_job), "message", why);
        record_event(why);
        return;
    }
    state_.pending = job;
    json_set(data, "status", "running");
    json_set(data, "sent_at", now());
    json_set(data, "deadline", now() + 8);
    json_set(data, "ack", false);
    if (action == "indoor_profile" || action == "outdoor_profile" || action == "height_profile") {
        const bool heights = action == "height_profile";
        const auto& navigation_profile =
            action == "outdoor_profile" ? outdoor_profile() : indoor_profile();
        const auto& profile = heights ? height_profile() : navigation_profile;
        Json before = Json::object();
        Json changes = Json::array();
        for (const auto& [name, value] : profile) {
            const auto found = state_.params.find(name);
            if (found == state_.params.end()) {
                key_error(name);
            }
            json_set(before, name, found->second);
        }
        for (const auto& [name, value] : profile) {
            if (!python_equal(state_.params.at(name), value)) {
                changes.push_back(Json::array({name, profile_value(value, !heights)}));
            }
        }
        json_set(data, "stage", "profile");
        json_set(data, "deadline", now() + 30);
        json_set(data, "before", before);
        json_set(data, "changes", changes);
        advance_profile(job);
        return;
    }
    if (action == "arm") {
        const std::optional<int> latest = state_.fc_events->latest();
        json_set(data, "event_sequence_before", latest ? Json(*latest) : Json());
    }
    if (action == "takeoff") {
        state_.holding = false;
        const Json* position = sample("LOCAL_POSITION_NED");
        const auto current_height = height().first;
        if (!current_height || position == nullptr) {
            finish("rejected", "높이·위치 정보가 유효하지 않습니다");
            return;
        }
        const double current = *current_height;
        const double wanted = as_double(at_key(data, "height"));
        const double distance = wanted - current;
        if (distance <= limits_.target_tolerance) {
            finish("rejected", "목표 높이가 현재 지면 높이보다 충분히 높지 않습니다");
            return;
        }
        const Json* attitude = sample("ATTITUDE");
        if (attitude == nullptr) {
            finish("rejected", "자세 정보가 유효하지 않습니다");
            return;
        }
        const double start_z = as_double(at_key(*position, "z"));
        state_.local_target = std::make_shared<Json>(Json{{"x", at_key(*position, "x")},
                                                          {"y", at_key(*position, "y")},
                                                          {"start_z", at_key(*position, "z")},
                                                          {"target_z", start_z - distance},
                                                          {"distance", distance},
                                                          {"height", at_key(data, "height")},
                                                          {"yaw", at_key(*attitude, "yaw")},
                                                          {"climb_at", nullptr}});
        state_.owned_flight = true;
        state_.owned_mode_confirmed = false;
        json_set(data, "stage", "priming");
        json_set(data, "prime_until", now() + 1.3);
        local_setpoint();
    } else if (action == "set_height") {
        const Json* position = sample("LOCAL_POSITION_NED");
        const auto current_height = height().first;
        if (!current_height || position == nullptr) {
            finish("rejected", "높이·위치 정보가 유효하지 않습니다");
            return;
        }
        const double current = *current_height;
        if (!state_.local_target) {
            finish("rejected", "고도 유지 목표가 없습니다");
            return;
        }
        const double start_z = ramped_z();
        const double wanted = as_double(at_key(data, "height"));
        const double target_z = as_double(at_key(*position, "z")) - (wanted - current);
        const double distance = start_z - target_z;
        // 원래 수평·yaw 목표(드리프트 감시 원점 포함)는 그대로 둔다.
        Json& target = *state_.local_target;
        json_set(target, "start_z", start_z);
        json_set(target, "target_z", target_z);
        json_set(target, "distance", distance);
        json_set(target, "height", at_key(data, "height"));
        json_set(target, "climb_at", now());
        json_set(data, "stage", "height");
        json_set(data, "command", nullptr);
        json_set(data, "message", "목표 고도 " + python_format_fixed(wanted, 1) + "m로 이동 중");
        json_set(data, "deadline",
                 now() + std::max(limits_.takeoff_timeout,
                                  (std::abs(distance) / limits_.climb_speed) + 8));
        local_setpoint();
        record_event("목표 고도 " + python_format_fixed(wanted, 1) +
                     "m로 변경 — 실제 도달 확인 중");
    } else if (action == "offboard") {
        state_.holding = false;
        state_.local_target.reset();
        json_set(data, "stage", "priming");
        json_set(data, "prime_until", now() + 1.3);
        zero();
    } else {
        issue(job);
    }
}

void FlightLink::issue(const Job& job) {
    Json& data = *job;
    const std::string action = at_key(data, "action").get<std::string>();
    json_set(data, "stage", "command");
    json_set(data, "sent_at", now());
    json_set(data, "ack", false);
    json_set(data, "deadline", now() + 8);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (action == "arm" || action == "disarm") {
        json_set(data, "command", kCmdArmDisarm);
        send_command(kCmdArmDisarm, {action == "arm" ? 1.0 : 0.0, 0});
    } else if (action == "takeoff") {
        json_set(data, "command", kCmdDoSetMode);
        json_set(data, "wanted_mode", "offboard");
        state_.owned_flight = true;
        state_.owned_mode_confirmed = false;
        send_mode("offboard");
    } else if (action == "land") {
        state_.local_target.reset();
        state_.external = false;
        state_.owned_flight = state_.owned_mode_confirmed = false;
        state_.holding = false;
        json_set(data, "command", kCmdNavLand);
        state_.landing = true;
        state_.landing_mode_seen = false;
        send_command(kCmdNavLand, {0, 0, 0, nan, nan, nan, nan});
    } else if (action == "set_home") {
        json_set(data, "command", kCmdDoSetHome);
        send_command(kCmdDoSetHome, {1, 0, 0, nan, nan, nan, nan});
    } else if (action == "yaw_handoff") {
        // FC가 Offboard 아닌 모드를 확인할 때까지 기존 위치·고도 출력을 유지한다.
        // 인계가 거부되어도 hover 출력이 끊기지 않아야 한다.
        state_.external = true;
        json_set(data, "handoff_previous_holding", state_.holding);
        const std::string wanted =
            environment() == std::optional<std::string>("indoor") ? "position" : "hold";
        json_set(data, "command", kCmdDoSetMode);
        json_set(data, "wanted_mode", wanted);
        json_set(data, "deadline", now() + 3);
        send_mode(wanted);
    } else if (action == "hold" && mode() == std::optional<std::string>("offboard") &&
               (command_owner_.path().empty() || command_owner_.held())) {
        state_.local_target.reset();
        state_.external = state_.owned_flight = true;
        state_.holding = true;
        zero();
        json_set(data, "command", nullptr);
        json_set(data, "ack", true);
        json_set(data, "wanted_mode", "offboard");
    } else if (action == "reboot") {
        json_set(data, "command", kCmdReboot);
        send_command(kCmdReboot, {1, 0});
    } else {
        const std::string wanted = action == "manual" ? "position" : action;
        json_set(data, "command", kCmdDoSetMode);
        json_set(data, "wanted_mode", wanted);
        send_mode(wanted);
    }
}

bool FlightLink::observed(const Json& job) const {
    const std::string action = at_key(job, "action").get<std::string>();
    if (action == "set_height") {
        const auto& target = state_.local_target;
        const std::optional<double> current = height().first;
        if (!target || !reason("set_height").empty()) {
            return false;
        }
        const Json climb_at = at_key(*target, "climb_at");
        if (climb_at.is_null() ||
            now() - as_double(climb_at) <
                std::abs(as_double(at_key(*target, "distance"))) / limits_.climb_speed) {
            return false;
        }
        const auto found = state_.samples.find(height_sample_kind());
        const double received = found == state_.samples.end() ? -kInfinity : found->second.at;
        return received > as_double(at_key(job, "sent_at")) && current &&
               std::abs(*current - as_double(at_key(job, "height"))) <= limits_.target_tolerance;
    }
    if (!python_truthy(at_key(job, "ack"))) {
        return false;
    }
    const auto heartbeat_at = [this]() {
        const auto found = state_.samples.find("HEARTBEAT");
        if (found == state_.samples.end()) {
            key_error("HEARTBEAT");
        }
        return found->second.at;
    };
    if (action == "arm" || action == "disarm") {
        return fresh("HEARTBEAT", 2.5) && heartbeat_at() >= as_double(at_key(job, "sent_at")) &&
               armed() == (action == "arm");
    }
    if (action == "takeoff") {
        const std::optional<double> current = height().first;
        return armed() && ground() == false && mode() == std::optional<std::string>("offboard") &&
               current && *current >= as_double(at_key(job, "height")) - limits_.takeoff_tolerance;
    }
    if (action == "land") {
        return mode() == std::optional<std::string>("land") || (ground() == true && !armed());
    }
    if (action == "set_home") {
        const auto found = state_.samples.find("HOME_POSITION");
        const Json ack_at = get_or_null(&job, "ack_at");
        return found != state_.samples.end() &&
               found->second.at >= (ack_at.is_null() ? kInfinity : as_double(ack_at));
    }
    const Json wanted = get_or_null(&job, "wanted_mode");
    const std::optional<std::string> current = mode();
    const bool same = current ? wanted == Json(*current) : wanted.is_null();
    return same && heartbeat_at() >= as_double(at_key(job, "sent_at"));
}

void FlightLink::ingest(const InboundMessage& message) {
    const std::string& kind = message.kind;
    if (message.system != 1 || message.component != 1) {
        return;
    }
    const Json& data = message.data;
    const double received = now();
    if (state_.follow_b) {
        state_.follow_b->ingest(kind, data);
    }
    if (kind == "EVENT" || kind == "CURRENT_EVENT_SEQUENCE" || kind == "RESPONSE_EVENT_ERROR") {
        state_.fc_events->ingest(kind, data);
        const Json* event = state_.fc_events->last_record();
        if (kind == "EVENT" && event != nullptr) {
            const Job job = state_.pending ? state_.pending : state_.last_job;
            const Json previous = job ? get_or_null(job.get(), "event_sequence_before") : Json();
            const Json sequence = get_or_null(event, "sequence");
            const bool is_new = !previous.is_null() && !sequence.is_null() &&
                                ((as_int(sequence) - as_int(previous)) & 65535) > 0 &&
                                ((as_int(sequence) - as_int(previous)) & 65535) < 32768;
            const Json text = get_or_null(event, "text");
            const std::string text_value = text.is_string() ? text.get<std::string>() : "";
            if (job && get_or_null(job.get(), "action") == Json("arm") &&
                !python_truthy(get_or_null(event, "history")) && is_new &&
                python_truthy(get_or_null(event, "decoded")) &&
                text_value.rfind("Arming denied:", 0) == 0) {
                const Json sent = get_or_null(job.get(), "sent_at");
                const Json deadline = get_or_null(job.get(), "deadline");
                const double at = as_double(at_key(*event, "at"));
                if ((sent.is_null() ? kInfinity : as_double(sent)) <= at &&
                    at <= (deadline.is_null() ? -kInfinity : as_double(deadline))) {
                    json_set((*job), "fc_reason", text_value);
                    if (get_or_null(job.get(), "status") == Json("rejected") &&
                        (*job).at("message").get<std::string>().find(text_value) ==
                            std::string::npos) {
                        json_set((*job), "message",
                                 (*job).at("message").get<std::string>() + " · " + text_value);
                    }
                }
            }
            // 2026-11-18 이륙 조건 보강: 이번 메시지로 새로 남은 기록만 자기장 경고 판정에 넣는다
            const Json id = get_or_null(event, "id");
            if (!id.is_null() && !sequence.is_null() &&
                as_int(sequence) == as_int(data.at("sequence")) &&
                as_int(get_or_null(event, "epoch")) == state_.fc_events->epoch()) {
                const auto event_id = id.get<std::uint32_t>();
                record_mag_change(mag_interference_.observe(
                    state_.fc_events->epoch(), static_cast<int>(as_int(sequence)), event_id,
                    state_.fc_events->event_name(event_id),
                    as_int(get_or_null(event, "fc_time_ms")), known_armed(), now()));
            }
        }
    }
    // GPS 없는 FC는 SYSTEM_TIME을 보내지 않을 수 있다. FC 자체의 자세/위치 uptime도 확인한다.
    std::optional<std::int64_t> previous_boot;
    std::int64_t boot_ms = 0;
    bool rebooted = false;
    if ((kind == "SYSTEM_TIME" || kind == "ATTITUDE" || kind == "LOCAL_POSITION_NED" ||
         kind == "GLOBAL_POSITION_INT") &&
        data.contains("time_boot_ms")) {
        boot_ms = as_int(data.at("time_boot_ms"));
        const auto found = state_.boot_stamps.find(kind);
        if (found != state_.boot_stamps.end()) {
            previous_boot = found->second;
        }
        state_.boot_stamps[kind] = boot_ms;
        rebooted = previous_boot && *previous_boot - boot_ms > 1000 &&
                   *previous_boot - boot_ms < 0x80000000LL;
    }
    if (!rebooted && (kind == "ATTITUDE" || kind == "LOCAL_POSITION_NED") && previous_boot &&
        boot_ms <= *previous_boot) {
        return;
    }
    if (rebooted) {
        state_.boot_stamps.clear();
        state_.boot_stamps[kind] = boot_ms;
        state_.fc_events->reset("FC 재부팅 감지 · 이전 이벤트는 과거 기록");
        state_.fc_events->set_verification(false, "FC 재부팅 · 메타데이터 재확인 대기");
        mag_interference_.reset(state_.fc_events->epoch());
        state_.profile_reboot_required = false;
        store_profile_reboot_requirement();
        state_.uid.reset();
        state_.firmware.reset();
        state_.samples.clear();
        state_.sensor_stamps.clear();
        for (auto& [name, rows] : state_.pose_history) {
            rows.clear();
        }
        state_.pose_epoch += 1;
        command_owner_.release();
        state_.params.clear();
        state_.param_types.clear();
        state_.config_requested = false;
        state_.local_target.reset();
        state_.external = state_.owned_flight = false;
        if (state_.pending && get_or_null(state_.pending.get(), "action") == Json("reboot")) {
            finish("done", "FC 재부팅 확인됨 — 센서/파라미터 재확인 중");
        } else {
            cancel("FC 재부팅 — 진행 요청 취소");
        }
        state_.samples.erase("COMPONENT_INFORMATION");
        send_command(kCmdRequestMessage, {kMsgComponentInformation});
    }
    if (kind == "HEARTBEAT" && as_int(at_key(data, "autopilot")) != kAutopilotPx4) {
        return;
    }
    if (kind == "HEARTBEAT") {
        // 2026-11-18: 자기장 보고 판정은 보고보다 늦게 도착한 FC heartbeat가 필요하다
        mag_interference_.observe_heartbeat((as_int(at_key(data, "base_mode")) & kSafetyArmed) != 0,
                                            received);
    }
    if (kind == "DISTANCE_SENSOR" && as_int(at_key(data, "orientation")) != kPitch270) {
        return;
    }
    if (kind == "DISTANCE_SENSOR" || kind == "OPTICAL_FLOW_RAD") {
        Json stamp = get_or_null(&data, "time_boot_ms");
        if (!data.contains("time_boot_ms")) {
            stamp = get_or_null(&data, "time_usec");
        }
        const auto previous = state_.sensor_stamps.find(kind);
        if (stamp.is_null() || (previous != state_.sensor_stamps.end() &&
                                as_double(stamp) <= as_double(previous->second))) {
            return;
        }
        state_.sensor_stamps[kind] = stamp;
    }
    state_.samples[kind] = Sample{data, received};
    const auto history = state_.pose_history.find(kind);
    if (history != state_.pose_history.end() && all_floats_finite(data)) {
        history->second.push_back(
            {{"received_monotonic_ns", static_cast<std::int64_t>(received * 1e9)}, {"data", data}});
        while (history->second.size() > kMaxPoseHistory) {
            history->second.pop_front();
        }
    }
    if (kind == "AUTOPILOT_VERSION") {
        const std::optional<std::string> previous_uid = state_.uid;
        Bytes uid;
        for (const Json& value :
             get_or_null(&data, "uid2").is_array() ? data.at("uid2") : Json::array()) {
            uid.push_back(static_cast<std::uint8_t>(as_int(value)));
        }
        state_.uid = hex_lower(uid);
        const std::int64_t caps = as_int(at_key(data, "capabilities"));
        state_.param_bytewise =
            (caps & kParamEncodeBytewise) != 0 || (caps & kParamEncodeCCast) == 0;
        const std::int64_t version = as_int(at_key(data, "flight_sw_version"));
        state_.firmware = std::to_string(version >> 24) + "." +
                          std::to_string((version >> 16) & 255) + "." +
                          std::to_string((version >> 8) & 255);
        if (state_.uid != previous_uid) {
            record_event(state_.uid == expected_uid_ ? "FC UID 확인"
                                                     : "FC UID 불일치/미확인 — 제어 차단");
        }
        if (state_.uid != expected_uid_) {
            const bool booting = (state_.uid->empty() || *state_.uid == std::string(36, '0')) &&
                                 state_.pending &&
                                 get_or_null(state_.pending.get(), "action") == Json("reboot") &&
                                 python_truthy(get_or_null(state_.pending.get(), "ack"));
            if (!booting) {
                cancel("FC UID 불일치 — 진행 요청 취소");
            }
            state_.external = state_.owned_flight = false;
            state_.local_target.reset();
        }
        const Json* information = sample("COMPONENT_INFORMATION");
        state_.fc_events->verify(data, information != nullptr ? *information : Json());
    } else if (kind == "COMPONENT_INFORMATION") {
        const Json* version = sample("AUTOPILOT_VERSION");
        state_.fc_events->verify(version != nullptr ? *version : Json(), data);
    } else if (kind == "PARAM_VALUE") {
        std::string name = at_key(data, "param_id").get<std::string>();
        while (!name.empty() && name.back() == '\0') {
            name.pop_back();
        }
        const int type = static_cast<int>(as_int(at_key(data, "param_type")));
        Json value = optional_number(python_number(at_key(data, "param_value")));
        if (state_.param_bytewise && (type == kParamInt32 || type == kParamUint32)) {
            const float raw = float_of(as_double(at_key(data, "param_value")));
            std::uint32_t bits = 0;
            std::memcpy(&bits, &raw, sizeof(bits));
            value = type == kParamInt32 ? Json(static_cast<std::int32_t>(bits)) : Json(bits);
        }
        if (is_profile_param(name) && !value.is_null()) {
            state_.params[name] = value;
            state_.param_types[name] = type;
        }
        if (state_.pending && get_or_null(state_.pending.get(), "stage") == Json("profile") &&
            get_or_null(state_.pending.get(), "waiting_param") == Json(name)) {
            Json& pending = *state_.pending;
            if (!value.is_null() && python_equal(value, as_double(pending.at("waiting_value"))) &&
                type == as_int(pending.at("waiting_type"))) {
                if (name == "SYS_HAS_GPS" || name == "SYS_HAS_MAG" || name == "EKF2_HGT_REF" ||
                    name == "SYS_HAS_NUM_OF" || name == "SYS_HAS_NUM_DIST") {
                    state_.profile_reboot_required = true;
                    store_profile_reboot_requirement();
                }
                pending.at("changes").erase(pending.at("changes").begin());
                advance_profile(state_.pending);
            }
        }
    } else if (kind == "COMMAND_ACK" && state_.pending &&
               get_or_null(state_.pending.get(), "command") == at_key(data, "command")) {
        const Json target_system = get_or_null(&data, "target_system");
        const Json target_component = get_or_null(&data, "target_component");
        const std::int64_t system = target_system.is_null() ? 0 : as_int(target_system);
        const std::int64_t component = target_component.is_null() ? 0 : as_int(target_component);
        if ((system != 0 && system != 245) || (component != 0 && component != 191)) {
            return;
        }
        const std::int64_t result = as_int(at_key(data, "result"));
        Json& pending = *state_.pending;
        json_set(pending, "fc_result", result);
        if (result == kResultAccepted) {
            json_set(pending, "ack", true);
            json_set(pending, "ack_at", received);
            if (pending.at("action") == "set_home") {
                send_command(kCmdRequestMessage, {kMsgHomePosition});
            }
        } else if (result != kResultInProgress) {
            finish("rejected", "FC 명령 거부: " + result_name(result));
        }
    } else if (kind == "STATUSTEXT") {
        std::string text = at_key(data, "text").get<std::string>();
        while (!text.empty() && text.back() == '\0') {
            text.pop_back();
        }
        record_event(text);
    }
}

Json FlightLink::cmc_stream_status() const {
    const auto now_ns = static_cast<std::int64_t>(now() * 1e9);
    Json actual = Json::object();
    for (const auto& [kind, history] : state_.pose_history) {
        std::vector<const Json*> rows;
        for (const Json& row : history) {
            const std::int64_t delta = now_ns - row.at("received_monotonic_ns").get<std::int64_t>();
            if (delta >= 0 && delta <= 1'000'000'000) {
                rows.push_back(&row);
            }
        }
        Json arrival;
        Json source;
        if (rows.size() > 1) {
            const double span =
                static_cast<double>(rows.back()->at("received_monotonic_ns").get<std::int64_t>() -
                                    rows.front()->at("received_monotonic_ns").get<std::int64_t>()) /
                1e9;
            if (span > 0) {
                arrival = static_cast<double>(rows.size() - 1) / span;
            }
            bool integers = true;
            for (const Json* row : rows) {
                const Json stamp = get_or_null(&row->at("data"), "time_boot_ms");
                integers = integers && stamp.is_number_integer() && as_int(stamp) >= 0 &&
                           as_int(stamp) <= 0xffffffffLL;
            }
            if (integers) {
                const std::int64_t delta = (as_int(rows.back()->at("data").at("time_boot_ms")) -
                                            as_int(rows.front()->at("data").at("time_boot_ms"))) &
                                           0xffffffffLL;
                if (delta > 0 && delta < 2000) {
                    source =
                        static_cast<double>(rows.size() - 1) * 1000 / static_cast<double>(delta);
                }
            }
        }
        json_set(actual, kind,
                 {{"received_hz", arrival}, {"measurement_hz", source}, {"samples", rows.size()}});
    }
    Json requested;
    if (state_.config_requested) {
        requested = {{"ATTITUDE", 60}, {"LOCAL_POSITION_NED", 50}, {"DISTANCE_SENSOR", 50}};
    }
    return {{"requested_hz", requested}, {"observed", actual}};
}

void FlightLink::configure() {
    for (const auto& [message_id, rate] : kStreams) {
        send_command(kCmdSetMessageInterval, {static_cast<double>(message_id), 1e6 / rate});
    }
    state_.cmc_stream_environment = environment();
    for (const char* name : kProfileParams) {
        state_.param_read_retry_at[name] = now();
    }
    state_.param_read_retry_at[kProfileParamsExtra.at(0)] = now();
    state_.next_param_read_retry = now() + .25;
    for (const char* name : kProfileParams) {
        write(state_.mav->param_request_read(1, 1, name, -1));
    }
    write(state_.mav->param_request_read(1, 1, kProfileParamsExtra.at(0), -1));
    send_command(kCmdRequestMessage, {kMsgComponentInformation});
    state_.config_requested = true;
}

void FlightLink::refresh_missing_params() {
    // 읽기 전용 재요청: 초당 최대 4회, 빠진 키마다 3초에 한 번. 유효한 캐시 값은 다시 읽지 않고
    // 진행 중인 비행 명령과 겨루지 않는다. 기존 UID·신선도·이륙 가드가 기준이다.
    if (!state_.config_requested || !state_.link || state_.uid != expected_uid_ ||
        !fresh("HEARTBEAT", 1.5) || state_.pending || !state_.jobs.empty() || state_.external ||
        state_.owned_flight || follow_busy()) {
        return;
    }
    const double current = now();
    if (current < state_.next_param_read_retry) {
        return;
    }
    std::vector<std::string> missing;
    const auto consider = [&](const char* name) {
        const auto found = state_.params.find(name);
        if (found == state_.params.end() || !python_number(found->second)) {
            missing.emplace_back(name);
        }
    };
    for (const char* name : kProfileParams) {
        consider(name);
    }
    consider(kProfileParamsExtra.at(0));
    if (missing.empty()) {
        return;
    }
    const auto retry_at = [this](const std::string& name) {
        const auto found = state_.param_read_retry_at.find(name);
        return found == state_.param_read_retry_at.end() ? -kInfinity : found->second;
    };
    // min(missing, key=...): 같은 값이면 앞의 것
    std::string name = missing.front();
    for (const std::string& candidate : missing) {
        if (retry_at(candidate) < retry_at(name)) {
            name = candidate;
        }
    }
    if (current - retry_at(name) < 3.0) {
        return;
    }
    write(state_.mav->param_request_read(1, 1, name, -1));
    state_.param_read_retry_at[name] = current;
    state_.next_param_read_retry = current + .25;
}

void FlightLink::supervise() {
    refresh_missing_params();
    const std::optional<std::string> where = environment();
    if (state_.config_requested && state_.uid == expected_uid_ && state_.link &&
        fresh("HEARTBEAT", 1.5) && where && where != state_.cmc_stream_environment &&
        !state_.pending && state_.jobs.empty()) {
        for (const int message_id : {30, 32, 132}) {
            double rate = 0;
            for (const auto& [id, value] : kStreams) {
                if (id == message_id) {
                    rate = value;
                }
            }
            send_command(kCmdSetMessageInterval, {static_cast<double>(message_id), 1e6 / rate});
        }
        state_.cmc_stream_environment = where;
        record_event(std::string("CMC FC 표본 요청: 자세60Hz·위치·거리50Hz · ") +
                     (*where == "indoor" ? "실내" : "실외"));
    }
    if (fresh("HEARTBEAT", limits_.heartbeat_ttl) && ground() == true && !armed()) {
        if (state_.pending) {
            const Json action = get_or_null(state_.pending.get(), "action");
            if (action == "takeoff" || action == "offboard" || action == "set_height") {
                finish("failed", "FC 무장 해제됨 — 이륙/외부제어 중단");
            }
        }
        state_.external = state_.owned_flight = state_.owned_mode_confirmed = state_.holding =
            false;
        state_.local_target.reset();
    }
    const std::optional<std::string> current_mode = mode();
    const bool is_offboard = current_mode == std::optional<std::string>("offboard");
    if (state_.landing && current_mode == std::optional<std::string>("land")) {
        state_.landing_mode_seen = true;
    }
    if (state_.landing_mode_seen && fresh("HEARTBEAT", 2.5) &&
        current_mode != std::optional<std::string>("land") && ground() == false) {
        state_.landing = state_.landing_mode_seen = false;
        record_event("FC/RC가 착륙 모드를 해제했습니다 — 현재 실제 모드를 확인하세요");
    }
    if (state_.landing && ground() == true && !armed()) {
        state_.landing = state_.owned_flight = state_.external = false;
        state_.landing_mode_seen = false;
        record_event("실제 지상·무장 해제 확인");
    }
    if (state_.external && !is_offboard) {
        state_.external = state_.owned_flight = false;
        state_.local_target.reset();
        state_.holding = false;
        record_event("FC/RC 모드 전환 — 외부 송신 종료, 자동 재진입 없음");
    }
    if (state_.owned_flight && state_.owned_mode_confirmed && !is_offboard) {
        state_.external = state_.owned_flight = false;
        state_.local_target.reset();
        cancel("FC/RC 제어권 전환 — 앱 비행 감시 해제");
    }
    if (state_.pending && get_or_null(state_.pending.get(), "action") == Json("set_height") &&
        (!is_offboard || !armed() || ground() == true || !state_.local_target || !state_.external ||
         !state_.owned_flight)) {
        finish("cancelled", "비행 상태/제어권 전환 — 고도 변경 확인 중단");
    }
    if (state_.pending && get_or_null(state_.pending.get(), "stage") == Json("priming")) {
        if (!height().first || !navigation_reason().empty() || !local_ok()) {
            finish("rejected", "외부제어 priming 중 센서/추정 만료");
        } else if (now() >= as_double(at_key(*state_.pending, "prime_until"))) {
            issue(state_.pending);
        }
    }
    const auto heartbeat_at = [this](double missing) {
        const auto found = state_.samples.find("HEARTBEAT");
        return found == state_.samples.end() ? missing : found->second.at;
    };
    const auto ack_at = [this]() {
        const Json value = get_or_null(state_.pending.get(), "ack_at");
        return value.is_null() ? kInfinity : as_double(value);
    };
    if (state_.pending && get_or_null(state_.pending.get(), "action") == Json("takeoff") &&
        python_truthy(get_or_null(state_.pending.get(), "ack")) &&
        mode() == std::optional<std::string>("offboard") && armed() && state_.local_target &&
        heartbeat_at(0) >= ack_at() && at_key(*state_.local_target, "climb_at").is_null()) {
        json_set((*state_.local_target), "climb_at", now());
        state_.external = state_.owned_mode_confirmed = true;
        json_set((*state_.pending), "deadline",
                 now() + std::max(limits_.takeoff_timeout,
                                  (as_double(at_key(*state_.local_target, "distance")) /
                                   limits_.climb_speed) +
                                      8));
    }
    if (state_.pending) {
        const Json stage = get_or_null(state_.pending.get(), "stage");
        if ((stage == "command" || stage == "height") && observed(*state_.pending)) {
            const std::string action = at_key(*state_.pending, "action").get<std::string>();
            if (action == "offboard" || action == "takeoff") {
                state_.external = state_.owned_flight = true;
                state_.owned_mode_confirmed = true;
            }
            if ((action == "hold" || action == "yaw_handoff" || action == "manual" ||
                 action == "position" || action == "altitude" || action == "stabilized" ||
                 action == "land") &&
                mode() != std::optional<std::string>("offboard")) {
                state_.external = state_.owned_flight = false;
                state_.local_target.reset();
                command_owner_.release();
            }
            finish("done",
                   action == "set_height"
                       ? "목표 고도 " +
                             python_format_fixed(as_double(at_key(*state_.pending, "height")), 1) +
                             "m 실제 도달 확인"
                       : action + ": FC 응답·실제 상태 반영 확인");
        }
    }
    // 2026-11-24 사용자 지시: 목표 도달을 못 봐도 떠서 상승이 멈췄으면 지금 높이 유지로 완료.
    if (state_.pending && get_or_null(state_.pending.get(), "stage") == Json("command") &&
        takeoff_climb_stalled()) {
        const double wanted = as_double(at_key(*state_.pending, "height"));
        const std::optional<double> reached = height().first;
        state_.external = state_.owned_flight = true;
        state_.owned_mode_confirmed = true;
        finish("done", "takeoff: 상승 멈춤 확인 · 목표 " + python_format_fixed(wanted, 2) +
                           "m, 실제 " + python_format_fixed(reached.value_or(0.0), 2) +
                           "m에서 유지");
    }
    if (state_.pending && get_or_null(state_.pending.get(), "action") == Json("takeoff") &&
        python_truthy(get_or_null(state_.pending.get(), "ack")) && heartbeat_at(0) >= ack_at() &&
        mode() != std::optional<std::string>("offboard")) {
        state_.owned_flight = false;
        state_.external = false;
        state_.local_target.reset();
        finish("cancelled", "FC/RC 수동 모드 인계 — 이륙 요청 상태 확인 중단");
    }
    if (state_.pending && now() > as_double(at_key(*state_.pending, "deadline"))) {
        const std::string action = at_key(*state_.pending, "action").get<std::string>();
        finish(action == "takeoff" || action == "set_height" ? "failed" : "unknown",
               "FC 응답/실제 반영 확인 시간 초과 — 성공 미확인");
        if (action == "takeoff" && armed()) {
            state_.jobs.push_front(std::make_shared<Json>(Json{
                {"action", "land"}, {"status", "queued"}, {"message", "이륙 확인 시간 초과"}}));
            state_.landing = true;
        } else if (action == "set_height" && armed() &&
                   mode() == std::optional<std::string>("offboard")) {
            state_.jobs.push_front(
                std::make_shared<Json>(Json{{"action", "hold"},
                                            {"status", "queued"},
                                            {"message", "고도 변경 확인 시간 초과"}}));
        }
    }
    if (!state_.owned_flight || state_.landing) {
        return;
    }
    const std::optional<double> current = height().first;
    std::string intervention;
    std::string why;
    if (!fresh("HEARTBEAT", limits_.heartbeat_ttl)) {
        intervention = "land";
        why = "FC heartbeat 만료";
    } else if (!current || !local_ok() || !navigation_reason().empty()) {
        intervention = "land";
        why = environment() == std::optional<std::string>("outdoor")
                  ? "GPS/레이저/로컬 추정 만료"
                  : "레이저/플로우/로컬 추정 만료";
    } else if (*current > limits_.ceiling) {
        intervention = "land";
        why = "고도 상한 초과";
    } else if (*current >= limits_.brake_height() && state_.local_target) {
        intervention = "hold";
        why = "고도 상한 접근 — 이륙/외부제어 중지";
    } else if (now() - state_.lease_at > limits_.lease_ttl && !state_.holding) {
        intervention = "hold";
        why = "GUI 제어 연결 만료";
    }
    if (intervention.empty()) {
        return;
    }
    cancel(why);
    auto& follow = state_.follow_b;
    if (intervention == "hold" && follow && follow->offboard_hover_enabled() &&
        mode() == std::optional<std::string>("offboard")) {
        if (follow->establish_hover(why, now() - state_.lease_at > limits_.lease_ttl)) {
            follow->invalidate("released");
            state_.last_job =
                std::make_shared<Json>(Json{{"id", new_id()},
                                            {"action", "hold"},
                                            {"status", "done"},
                                            {"message", why + " · Offboard 고도 유지"}});
            return;
        }
        // 유효하지 않은 FC/센서/권한으로는 맹목적 hover를 허락하지 않는다.
        // 기존 착륙 안전 경로를 유지하고 AUTO_LOITER로 대신하지 않는다.
        intervention = "land";
        why += " · 안전 고도 유지 조건 미충족";
    }
    state_.external = state_.owned_flight = false;
    if (state_.uid == expected_uid_) {
        state_.jobs.push_front(std::make_shared<Json>(Json{
            {"id", new_id()}, {"action", intervention}, {"status", "queued"}, {"message", why}}));
        if (intervention == "land") {
            state_.landing = true;
        }
        record_event(why);
    }
}

void FlightLink::close_link() {
    if (state_.follow_b) {
        state_.follow_b->clear_bench();
        state_.follow_b->invalidate("fc_disconnected");
    }
    state_.external = state_.owned_flight = false;
    state_.local_target.reset();
    command_owner_.release();
    state_.holding = false;
    if (state_.link) {
        state_.link->close();
        state_.link.reset();
    }
    state_.gcs_mav.reset();
    // 2026-11-18: 연결이 끊기면 자기장 경고 상태는 모름 (다음 연결의 보고로 다시 정한다)
    mag_interference_.reset(state_.fc_events->epoch());
}

void FlightLink::run_once() {
    if (state_.link) {
        const Bytes incoming = state_.link->read_available();
        if (!incoming.empty()) {
            state_.parser.feed(incoming);
        }
        for (int i = 0; i < kMaxMessagesPerStep; ++i) {
            std::optional<InboundMessage> message = state_.parser.next();
            if (!message) {
                break;
            }
            ingest(*message);
            if (!state_.link) {
                break;
            }
        }
        if (now() - state_.last_heartbeat_tx > 1) {
            write(state_.mav->heartbeat(kTypeOnboardController, kAutopilotInvalid, 0, 0,
                                        kStateActive));
            state_.last_heartbeat_tx = now();
        }
        if (state_.gcs_mav && state_.uid == expected_uid_ && gcs_active() &&
            now() - state_.last_gcs_tx >= 1) {
            write(state_.gcs_mav->heartbeat(kTypeGcs, kAutopilotInvalid, 0, 0, kStateActive));
            state_.last_gcs_tx = now();
        }
        if (fresh("HEARTBEAT", 2.5) && now() - state_.last_probe > 1) {
            send_command(kCmdRequestMessage, {kMsgAutopilotVersion});
            state_.last_probe = now();
        }
        if (state_.uid == expected_uid_ && !state_.config_requested) {
            configure();
        }
        update_mag_interference();
        supervise();
        const auto event_range = state_.fc_events->due();
        if (event_range && state_.uid == expected_uid_ && fresh("HEARTBEAT", 2.5)) {
            write(state_.mav->request_event(1, 1, static_cast<std::uint16_t>(event_range->first),
                                            static_cast<std::uint16_t>(event_range->second)));
        }
        const Json pending_action =
            state_.pending ? get_or_null(state_.pending.get(), "action") : Json();
        const Json pending_stage =
            state_.pending ? get_or_null(state_.pending.get(), "stage") : Json();
        const bool prime = state_.pending &&
                           (pending_action == "offboard" || pending_action == "takeoff") &&
                           (pending_stage == "priming" || pending_stage == "command");
        if ((state_.external || prime) && now() - state_.last_tx >= .05) {
            if (state_.uid == expected_uid_ && fresh("HEARTBEAT", 2.5) && height().first &&
                local_ok() && navigation_reason().empty()) {
                if (state_.local_target) {
                    local_setpoint();
                } else {
                    zero();
                }
            }
        }
        if (!state_.external && !prime && !follow_busy()) {
            command_owner_.release();
        }
    }
    if (state_.follow_b) {
        state_.follow_b->tick();
    }
    if (!state_.pending && !state_.jobs.empty()) {
        const Job job = state_.jobs.front();
        state_.jobs.pop_front();
        dispatch(job);
    }
}

void FlightLink::step() {
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    try {
        run_once();
    } catch (const std::exception& error) {
        finish("failed", std::string("FC 연결 오류: ") + error.what());
        close_link();
    }
}

void FlightLink::stop() {
    {
        const std::scoped_lock<std::recursive_mutex> guard(lock_);
        if (state_.link && (armed() || state_.owned_flight || state_.pending ||
                            !fresh("HEARTBEAT", 2.5) || ground() != true)) {
            throw ValueError("신선한 지상·무장 해제를 확인한 뒤 게이트웨이를 종료할 수 있습니다");
        }
    }
    quit_ = true;
    if (thread_.joinable()) {
        thread_.join();
    }
    const std::scoped_lock<std::recursive_mutex> guard(lock_);
    close_link();
}

}  // namespace argus::gateway
