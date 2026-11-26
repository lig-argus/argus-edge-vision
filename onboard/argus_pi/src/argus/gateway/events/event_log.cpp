#include "argus/gateway/events/include/event_log.hpp"

#include <lzma.h>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iterator>
#include <regex>
#include <stdexcept>

namespace argus::gateway {

namespace {

constexpr std::uintmax_t kMaxMetadataBytes = std::uintmax_t{5} * 1024U * 1024U;
constexpr std::uint64_t kLzmaMemoryLimit = 256ULL * 1024ULL * 1024ULL;
const std::array<const char*, 10> kLevels{"EMERGENCY", "ALERT", "CRITICAL", "ERROR",    "WARNING",
                                          "NOTICE",    "INFO",  "DEBUG",    "PROTOCOL", "DISABLED"};

class MetadataError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

std::string read_file(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw MetadataError("[Errno 2] No such file or directory: '" + path.string() + "'");
    }
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

int base64_value(char c) {
    if (c >= 'A' && c <= 'Z') {
        return c - 'A';
    }
    if (c >= 'a' && c <= 'z') {
        return c - 'a' + 26;
    }
    if (c >= '0' && c <= '9') {
        return c - '0' + 52;
    }
    if (c == '+') {
        return 62;
    }
    if (c == '/') {
        return 63;
    }
    return -1;
}

// base64.b64decode(text, validate=True)
Bytes base64_decode_strict(const std::string& text) {
    static const std::regex kPattern("[A-Za-z0-9+/]*={0,2}");
    if (!std::regex_match(text, kPattern)) {
        throw MetadataError("Non-base64 digit found");
    }
    const std::size_t padding = text.size() - text.find_last_not_of('=') - 1;
    const std::size_t data_chars =
        text.find('=') == std::string::npos ? text.size() : text.find('=');
    if (data_chars % 4 == 1) {
        throw MetadataError("Invalid base64-encoded string: number of data characters (" +
                            std::to_string(data_chars) + ") cannot be 1 more than a multiple of 4");
    }
    if ((data_chars + (text.find('=') == std::string::npos ? 0 : padding)) % 4 != 0) {
        throw MetadataError("Incorrect padding");
    }
    Bytes out;
    out.reserve((data_chars * 3) / 4);
    std::uint32_t buffer = 0;
    int bits = 0;
    for (std::size_t i = 0; i < data_chars; ++i) {
        buffer = (buffer << 6) | static_cast<std::uint32_t>(base64_value(text.at(i)));
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<std::uint8_t>((buffer >> bits) & 0xff));
        }
    }
    return out;
}

// lzma.LZMADecompressor(memlimit=256MiB).decompress(raw, max_length=5MiB)
std::string xz_decode(const Bytes& raw) {
    lzma_stream stream = LZMA_STREAM_INIT;
    if (lzma_auto_decoder(&stream, kLzmaMemoryLimit, 0) != LZMA_OK) {
        throw MetadataError("LZMA 디코더 준비 실패");
    }
    std::vector<std::uint8_t> out(kMaxMetadataBytes);
    stream.next_in = raw.data();
    stream.avail_in = raw.size();
    stream.next_out = out.data();
    stream.avail_out = out.size();
    bool eof = false;
    for (;;) {
        const std::size_t before_in = stream.avail_in;
        const std::size_t before_out = stream.avail_out;
        const lzma_ret result = lzma_code(&stream, LZMA_RUN);
        if (result == LZMA_STREAM_END) {
            eof = true;
            break;
        }
        if (result != LZMA_OK) {
            lzma_end(&stream);
            throw MetadataError(result == LZMA_FORMAT_ERROR
                                    ? "Input format not supported by decoder"
                                    : "Corrupt input data");
        }
        if (stream.avail_out == 0 ||
            (stream.avail_in == before_in && stream.avail_out == before_out)) {
            break;
        }
    }
    const std::size_t produced = out.size() - stream.avail_out;
    lzma_end(&stream);
    if (!eof || produced >= kMaxMetadataBytes) {
        throw MetadataError("이벤트 정의 크기 초과/압축 불완전");
    }
    return {out.begin(), std::next(out.begin(), static_cast<std::ptrdiff_t>(produced))};
}

Bytes bytes_from_hex(const std::string& hex) {
    Bytes out;
    for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    }
    return out;
}

nlohmann::json value_or_null(const nlohmann::json& object, const char* key) {
    if (!object.is_object()) {
        return nullptr;
    }
    const auto found = object.find(key);
    return found == object.end() ? nlohmann::json() : *found;
}

int as_int(const nlohmann::json& value) { return static_cast<int>(value.get<long long>()); }

}  // namespace

EventLog::EventLog(Clock clock, const std::filesystem::path& metadata_path)
    : clock_(std::move(clock)) {
    try {
        std::error_code error;
        const auto size = std::filesystem::file_size(metadata_path, error);
        if (error) {
            throw MetadataError("[Errno 2] No such file or directory: '" + metadata_path.string() +
                                "'");
        }
        if (size > kMaxMetadataBytes) {
            throw MetadataError("메타데이터 크기 초과");
        }
        metadata_ = nlohmann::json::parse(read_file(metadata_path));
        if (!metadata_.contains("events_xz_base64")) {
            throw MetadataError("'events_xz_base64'");
        }
        const Bytes raw = base64_decode_strict(metadata_.at("events_xz_base64").get<std::string>());
        const auto crc = static_cast<std::uint32_t>(
            ::crc32(0xffffffffUL, raw.data(), static_cast<uInt>(raw.size())) ^ 0xffffffffUL);
        if (!metadata_.contains("events_crc")) {
            throw MetadataError("'events_crc'");
        }
        if (metadata_.at("events_crc") != nlohmann::json(crc)) {
            throw MetadataError("이벤트 메타데이터 CRC 불일치");
        }
        auto parser = std::make_unique<EventParser>();
        parser->load_definitions(nlohmann::ordered_json::parse(xz_decode(raw)));
        parser_ = std::move(parser);
    } catch (const std::exception& error) {
        parser_.reset();
        status_ = std::string("메타데이터 사용 불가: ") + error.what();
    }
}

void EventLog::verify(const nlohmann::json& version, const nlohmann::json& info) {
    if (!parser_ || !python_truthy(version) || !python_truthy(info)) {
        return;
    }
    Bytes uid;
    for (const auto& value :
         value_or_null(version, "uid2").is_array() ? version.at("uid2") : nlohmann::json::array()) {
        uid.push_back(static_cast<std::uint8_t>(value.get<int>()));
    }
    verified_ =
        nlohmann::json(hex_lower(uid)) == value_or_null(metadata_, "fc_uid") &&
        value_or_null(version, "flight_sw_version") ==
            value_or_null(metadata_, "firmware_version") &&
        value_or_null(version, "flight_custom_version") ==
            value_or_null(metadata_, "firmware_custom_version") &&
        value_or_null(info, "general_metadata_file_crc") == value_or_null(metadata_, "general_crc");
    status_ = verified_ ? "FC 펌웨어·메타데이터 확인됨" : "FC 정의 불일치 · 원자료 표시";
    for (nlohmann::json& row : records_) {
        if (!row.at("id").is_null()) {
            row.update(
                decode(row.at("id").get<std::uint32_t>(), bytes_from_hex(row.at("arguments_hex"))));
        }
    }
}

void EventLog::reset(const std::string& reason) {
    epoch_ += 1;
    latest_.reset();
    pending_.clear();
    seen_.clear();
    records_.push_back({{"at", clock_()},
                        {"epoch", epoch_},
                        {"id", nullptr},
                        {"text", reason},
                        {"severity", "INFO"},
                        {"display", true}});
    if (records_.size() > kMaxRecords) {
        records_.pop_front();
    }
}

bool EventLog::observe_boot(std::int64_t milliseconds) {
    // SYSTEM_TIME의 역행과 32bit wrap을 구분한다. 과거 EVENT 시각으로 판단하지 않는다.
    const bool reboot =
        boot_ms_ && *boot_ms_ - milliseconds > 1000 && *boot_ms_ - milliseconds < 0x80000000LL;
    boot_ms_ = milliseconds;
    if (reboot) {
        reset("FC 재부팅 감지 · 이전 이벤트는 과거 기록");
        verified_ = false;
        status_ = "FC 재부팅 · 메타데이터 재확인 대기";
    }
    return reboot;
}

bool EventLog::seen(int sequence) const {
    return std::any_of(seen_.begin(), seen_.end(), [&](const std::pair<int, int>& item) {
        return item.first == epoch_ && item.second == sequence;
    });
}

EventLog::Pending* EventLog::find_pending(int sequence) {
    for (Pending& item : pending_) {
        if (item.sequence == sequence) {
            return &item;
        }
    }
    return nullptr;
}

void EventLog::request(int first, int last, bool history) {
    const int count = std::min(((last - first) & 65535) + 1, 32);
    first = (last - count + 1) & 65535;
    for (int i = 0; i < count; ++i) {
        const int sequence = (first + i) & 65535;
        if (!seen(sequence)) {
            const double now = clock_();  // 원본 setdefault처럼 이미 있어도 시각을 읽는다
            if (find_pending(sequence) == nullptr) {
                pending_.push_back({sequence, now, 0, history});
            }
        }
    }
    // 제한된 재요청: 큰 공백은 FC를 몰아붙이지 않고 빈 채로 보인다.
    while (pending_.size() > kMaxPending) {
        pending_.erase(pending_.begin());
        expired_ += 1;
    }
}

nlohmann::json EventLog::decode(std::uint32_t event_id, const Bytes& arguments) const {
    nlohmann::json result = {
        {"decoded", false}, {"text", "미해석 이벤트"}, {"group", "unknown"}, {"description", ""}};
    if (!verified_ || !parser_) {
        result.at("text") = std::string("미해석 이벤트 · ") + status_;
        return result;
    }
    try {
        const std::optional<ParsedEventText> event = parser_->parse(event_id, arguments);
        if (event) {
            result.at("decoded") = true;
            result.at("text") = utf8_prefix(event->message, 2000);
            result.at("group") = event->group;
            result.at("description") = utf8_prefix(event->description, 6000);
        }
    } catch (const EventDecodeError& error) {
        result.at("text") = std::string("미해석 이벤트 · ") + error.what();
    }
    return result;
}

std::string EventLog::event_name(std::uint32_t event_id) const {
    if (!verified_ || !parser_) {
        return "";
    }
    return parser_->name(event_id).value_or("");
}

bool EventLog::is_pending(int sequence) const {
    return std::any_of(pending_.begin(), pending_.end(),
                       [&](const Pending& item) { return item.sequence == (sequence & 65535); });
}

void EventLog::ingest(const std::string& kind, const nlohmann::json& data) {
    if (kind == "CURRENT_EVENT_SEQUENCE") {
        const int sequence = as_int(data.at("sequence"));
        const bool reset_now = (data.at("flags").get<long long>() & 1) != 0;
        if (reset_now && !reset_flag_ && latest_) {
            reset("FC 이벤트 순번 초기화 · 이전 기록과 구분");
        }
        reset_flag_ = reset_now;
        if (!latest_) {
            latest_ = sequence;
            if (!reset_now) {
                request((sequence - 31) & 65535, sequence, true);
            }
        } else {
            const int delta = (sequence - *latest_) & 65535;
            if (delta > 0 && delta < 32768) {
                request((*latest_ + 1) & 65535, sequence, false);
                latest_ = sequence;
            }
        }
        return;
    }
    const auto in_targets = [](const nlohmann::json& value, int first, int second) {
        return value.is_number_integer() &&
               (value.get<long long>() == first || value.get<long long>() == second);
    };
    if (kind == "RESPONSE_EVENT_ERROR") {
        if (in_targets(value_or_null(data, "target_system"), 0, 245) &&
            in_targets(value_or_null(data, "target_component"), 0, 191)) {
            const int sequence = as_int(data.at("sequence"));
            for (auto it = pending_.begin(); it != pending_.end(); ++it) {
                if (it->sequence == sequence) {
                    pending_.erase(it);
                    unavailable_ += 1;
                    break;
                }
            }
        }
        return;
    }
    if (kind != "EVENT") {
        return;
    }
    const int sequence = as_int(data.at("sequence"));
    bool history = false;
    for (auto it = pending_.begin(); it != pending_.end(); ++it) {
        if (it->sequence == sequence) {
            history = it->history;
            pending_.erase(it);
            break;
        }
    }
    if (seen(sequence)) {
        return;
    }
    seen_.emplace_back(epoch_, sequence);
    if (seen_.size() > kMaxSeen) {
        seen_.pop_front();
    }
    if (!latest_) {
        latest_ = sequence;
    } else {
        const int delta = (sequence - *latest_) & 65535;
        if (delta > 0 && delta < 32768) {
            if (delta > 1) {
                request((*latest_ + 1) & 65535, (sequence - 1) & 65535, false);
            }
            latest_ = sequence;
        }
    }
    const auto destination_ok = [&](const char* key, int other) {
        const auto found = data.find(key);
        return found == data.end() || in_targets(*found, 0, other);
    };
    if (!destination_ok("destination_system", 245) ||
        !destination_ok("destination_component", 191)) {
        return;
    }
    Bytes arguments;
    for (const auto& value : data.at("arguments")) {
        arguments.push_back(static_cast<std::uint8_t>(value.get<int>()));
    }
    const int level = static_cast<int>(data.at("log_levels").get<long long>() & 15);
    nlohmann::json row = {
        {"at", clock_()},
        {"epoch", epoch_},
        {"id", data.at("id").get<std::uint32_t>()},
        {"sequence", sequence},
        {"fc_time_ms", data.at("event_time_boot_ms").get<long long>()},
        {"severity", level < static_cast<int>(kLevels.size())
                         ? std::string(kLevels.at(static_cast<std::size_t>(level)))
                         : std::to_string(level)},
        {"display", level < 8},
        {"history", history},
        {"arguments_hex", hex_lower(arguments)},
    };
    row.update(decode(row.at("id").get<std::uint32_t>(), arguments));
    records_.push_back(std::move(row));
    if (records_.size() > kMaxRecords) {
        records_.pop_front();
    }
}

std::optional<std::pair<int, int>> EventLog::due() {
    const double now = clock_();
    std::vector<int> ready;
    for (auto it = pending_.begin(); it != pending_.end();) {
        if (now < it->due) {
            ++it;
            continue;
        }
        if (it->attempts >= 3) {
            it = pending_.erase(it);
            expired_ += 1;
            continue;
        }
        if (!ready.empty() && it->sequence != ((ready.back() + 1) & 65535)) {
            break;
        }
        ready.push_back(it->sequence);
        it->attempts += 1;
        it->due = now + 1;
        ++it;
    }
    if (ready.empty()) {
        return std::nullopt;
    }
    return std::make_pair(ready.front(), ready.back());
}

nlohmann::json EventLog::snapshot() const {
    nlohmann::json records = nlohmann::json::array();
    for (const nlohmann::json& row : records_) {
        if (row.value("display", false)) {
            records.push_back(row);
        }
    }
    return {{"metadata", status_},         {"epoch", epoch_},     {"pending", pending_.size()},
            {"unavailable", unavailable_}, {"expired", expired_}, {"records", records}};
}

nlohmann::json EventLog::debug_state() const {
    nlohmann::json pending = nlohmann::json::array();
    for (const Pending& item : pending_) {
        pending.push_back({item.sequence, item.due, item.attempts, item.history});
    }
    nlohmann::json seen_list = nlohmann::json::array();
    for (const auto& [epoch, sequence] : seen_) {
        seen_list.push_back({epoch, sequence});
    }
    return {{"latest", latest_ ? nlohmann::json(*latest_) : nlohmann::json()},
            {"epoch", epoch_},
            {"boot_ms", boot_ms_ ? nlohmann::json(*boot_ms_) : nlohmann::json()},
            {"reset_flag", reset_flag_},
            {"unavailable", unavailable_},
            {"expired", expired_},
            {"verified", verified_},
            {"status", status_},
            {"pending", pending},
            {"seen", seen_list},
            {"records", records_.size()}};
}

}  // namespace argus::gateway
