#include "argus/gateway/common/include/python_compat.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <regex>
#include <string>

namespace argus::gateway {

namespace {

constexpr char32_t kReplacementCharacter = 0xfffd;  // 잘못된 UTF-8 바이트 대신 넣는 문자

constexpr std::array<char, 16> kHexDigits{'0', '1', '2', '3', '4', '5', '6', '7',
                                          '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

// Python json의 역슬래시-u 이스케이프 (소문자 16진수 4자리)
void append_u_escape(std::string& out, unsigned value) {
    out += '\\';
    out += 'u';
    for (int shift = 12; shift >= 0; shift -= 4) {
        out += kHexDigits.at((value >> static_cast<unsigned>(shift)) & 0x0fU);
    }
}

void dump_string(std::string& out, const std::string& text) {
    out += '"';
    for (const char32_t code : utf8_to_u32(text)) {
        switch (code) {
            case U'"':
                out += "\\\"";
                continue;
            case U'\\':
                out += "\\\\";
                continue;
            case U'\n':
                out += "\\n";
                continue;
            case U'\r':
                out += "\\r";
                continue;
            case U'\t':
                out += "\\t";
                continue;
            case U'\b':
                out += "\\b";
                continue;
            case U'\f':
                out += "\\f";
                continue;
            default:
                break;
        }
        if (code >= 0x20 && code <= 0x7e) {
            out += static_cast<char>(code);
        } else if (code < 0x10000) {
            append_u_escape(out, static_cast<unsigned>(code));
        } else {
            const char32_t reduced = code - 0x10000;
            append_u_escape(out, static_cast<unsigned>(0xd800 + (reduced >> 10)));
            append_u_escape(out, static_cast<unsigned>(0xdc00 + (reduced & 0x3ff)));
        }
    }
    out += '"';
}

// JSON 중첩만큼 재귀한다. 입력은 배포 설정 파일(깊이 수 단계)뿐이라 재귀 깊이가 작다.
void dump_value(std::string& out, const nlohmann::json& value) {  // NOLINT(misc-no-recursion)
    switch (value.type()) {
        case nlohmann::json::value_t::null:
            out += "null";
            return;
        case nlohmann::json::value_t::boolean:
            out += value.get<bool>() ? "true" : "false";
            return;
        case nlohmann::json::value_t::number_integer:
            out += std::to_string(value.get<std::int64_t>());
            return;
        case nlohmann::json::value_t::number_unsigned:
            out += std::to_string(value.get<std::uint64_t>());
            return;
        case nlohmann::json::value_t::number_float: {
            const double number = value.get<double>();
            if (std::isnan(number)) {
                out += "NaN";
            } else if (std::isinf(number)) {
                out += number > 0 ? "Infinity" : "-Infinity";
            } else {
                out += python_float_repr(number);
            }
            return;
        }
        case nlohmann::json::value_t::string:
            dump_string(out, value.get_ref<const std::string&>());
            return;
        case nlohmann::json::value_t::array: {
            out += '[';
            bool first = true;
            for (const auto& item : value) {
                if (!first) {
                    out += ", ";
                }
                first = false;
                dump_value(out, item);
            }
            out += ']';
            return;
        }
        case nlohmann::json::value_t::object: {
            // nlohmann::json 객체는 std::map이라 키가 이미 바이트(=코드 포인트) 순으로 정렬돼 있다.
            out += '{';
            bool first = true;
            for (const auto& [key, item] : value.items()) {
                if (!first) {
                    out += ", ";
                }
                first = false;
                dump_string(out, key);
                out += ": ";
                dump_value(out, item);
            }
            out += '}';
            return;
        }
        default:
            throw std::invalid_argument("JSON 값 형식 미지원");
    }
}

}  // namespace

std::string python_float_repr(double value) {
    if (std::isnan(value)) {
        return "nan";
    }
    if (std::isinf(value)) {
        return value > 0 ? "inf" : "-inf";
    }
    if (value == 0.0) {
        return std::signbit(value) ? "-0.0" : "0.0";
    }
    std::array<char, 64> buffer{};
    const auto result = std::to_chars(buffer.data(), std::next(buffer.data(), buffer.size()), value,
                                      std::chars_format::scientific);
    std::string text(buffer.data(), result.ptr);
    const bool negative = text.front() == '-';
    if (negative) {
        text.erase(0, 1);
    }
    const auto exponent_at = text.find('e');
    const int exponent = std::stoi(text.substr(exponent_at + 1));
    std::string digits;
    for (const char c : text.substr(0, exponent_at)) {
        if (c != '.') {
            digits += c;
        }
    }
    while (digits.size() > 1 && digits.back() == '0') {
        digits.pop_back();
    }
    const int point = exponent + 1;
    std::string out;
    if (point <= -4 || point > 16) {
        out = digits.substr(0, 1);
        if (digits.size() > 1) {
            out += "." + digits.substr(1);
        }
        const int magnitude = std::abs(exponent);
        out += exponent < 0 ? "e-" : "e+";
        if (magnitude < 10) {
            out += '0';
        }
        out += std::to_string(magnitude);
    } else if (point <= 0) {
        out = "0." + std::string(static_cast<std::size_t>(-point), '0') + digits;
    } else if (static_cast<std::size_t>(point) >= digits.size()) {
        out = digits + std::string(static_cast<std::size_t>(point) - digits.size(), '0') + ".0";
    } else {
        out = digits.substr(0, static_cast<std::size_t>(point)) + "." +
              digits.substr(static_cast<std::size_t>(point));
    }
    return negative ? "-" + out : out;
}

std::string python_json_dumps_sorted(const nlohmann::json& value) {
    std::string out;
    dump_value(out, value);
    return out;
}

bool python_truthy(const nlohmann::json& value) {
    switch (value.type()) {
        case nlohmann::json::value_t::null:
            return false;
        case nlohmann::json::value_t::boolean:
            return value.get<bool>();
        case nlohmann::json::value_t::number_integer:
            return value.get<std::int64_t>() != 0;
        case nlohmann::json::value_t::number_unsigned:
            return value.get<std::uint64_t>() != 0;
        case nlohmann::json::value_t::number_float:
            return value.get<double>() != 0.0;
        case nlohmann::json::value_t::string:
            return !value.get_ref<const std::string&>().empty();  // empty()는 문자열에 항상 false
        case nlohmann::json::value_t::array:
        case nlohmann::json::value_t::object:
            return !value.empty();
        default:
            return true;
    }
}

std::u32string utf8_to_u32(const std::string& text) {
    std::u32string out;
    out.reserve(text.size());
    std::size_t i = 0;
    while (i < text.size()) {
        const auto lead = static_cast<unsigned char>(text.at(i));
        int extra = 0;
        char32_t code = 0;
        if (lead < 0x80) {
            code = lead;
        } else if ((lead & 0xe0) == 0xc0) {
            code = lead & 0x1f;
            extra = 1;
        } else if ((lead & 0xf0) == 0xe0) {
            code = lead & 0x0f;
            extra = 2;
        } else if ((lead & 0xf8) == 0xf0) {
            code = lead & 0x07;
            extra = 3;
        } else {
            out += kReplacementCharacter;
            ++i;
            continue;
        }
        if (i + static_cast<std::size_t>(extra) >= text.size()) {
            out += kReplacementCharacter;
            ++i;
            continue;
        }
        bool valid = true;
        for (int k = 1; k <= extra; ++k) {
            const auto next = static_cast<unsigned char>(text.at(i + static_cast<std::size_t>(k)));
            if ((next & 0xc0) != 0x80) {
                valid = false;
                break;
            }
            code = (code << 6) | (next & 0x3f);
        }
        if (!valid) {
            out += kReplacementCharacter;
            ++i;
            continue;
        }
        out += code;
        i += static_cast<std::size_t>(extra) + 1;
    }
    return out;
}

std::string u32_to_utf8(const std::u32string& text) {
    std::string out;
    out.reserve(text.size());
    for (const char32_t code : text) {
        if (code < 0x80) {
            out += static_cast<char>(code);
        } else if (code < 0x800) {
            out += static_cast<char>(0xc0 | (code >> 6));
            out += static_cast<char>(0x80 | (code & 0x3f));
        } else if (code < 0x10000) {
            out += static_cast<char>(0xe0 | (code >> 12));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3f));
            out += static_cast<char>(0x80 | (code & 0x3f));
        } else {
            out += static_cast<char>(0xf0 | (code >> 18));
            out += static_cast<char>(0x80 | ((code >> 12) & 0x3f));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3f));
            out += static_cast<char>(0x80 | (code & 0x3f));
        }
    }
    return out;
}

bool python_is_space(char32_t code_point) {
    return (code_point >= 0x09 && code_point <= 0x0d) ||
           (code_point >= 0x1c && code_point <= 0x20) || code_point == 0x85 || code_point == 0xa0 ||
           code_point == 0x1680 || (code_point >= 0x2000 && code_point <= 0x200a) ||
           code_point == 0x2028 || code_point == 0x2029 || code_point == 0x202f ||
           code_point == 0x205f || code_point == 0x3000;
}

std::u32string python_strip(const std::u32string& text) {
    std::size_t begin = 0;
    std::size_t end = text.size();
    while (begin < end && python_is_space(text.at(begin))) {
        ++begin;
    }
    while (end > begin && python_is_space(text.at(end - 1))) {
        --end;
    }
    return text.substr(begin, end - begin);
}

std::string utf8_prefix(const std::string& text, std::size_t code_points) {
    std::size_t count = 0;
    for (std::size_t i = 0; i < text.size(); ++i) {
        if ((static_cast<unsigned char>(text.at(i)) & 0xc0) != 0x80) {
            if (count == code_points) {
                return text.substr(0, i);
            }
            ++count;
        }
    }
    return text;
}

std::string hex_lower(const Bytes& bytes) {
    std::string out;
    out.reserve(bytes.size() * 2);
    for (const std::uint8_t byte : bytes) {
        out += kHexDigits.at(byte >> 4);
        out += kHexDigits.at(byte & 0x0f);
    }
    return out;
}

bool python_parse_int(const std::string& text, long long& value) {
    std::size_t begin = 0;
    std::size_t end = text.size();
    while (begin < end && python_is_space(static_cast<unsigned char>(text.at(begin)))) {
        ++begin;
    }
    while (end > begin && python_is_space(static_cast<unsigned char>(text.at(end - 1)))) {
        --end;
    }
    bool negative = false;
    if (begin < end && (text.at(begin) == '+' || text.at(begin) == '-')) {
        negative = text.at(begin) == '-';
        ++begin;
    }
    if (begin == end) {
        return false;
    }
    std::string digits;
    bool previous_digit = false;
    for (std::size_t i = begin; i < end; ++i) {
        const char c = text.at(i);
        if (c >= '0' && c <= '9') {
            digits += c;
            previous_digit = true;
        } else if (c == '_' && previous_digit && i + 1 < end && text.at(i + 1) >= '0' &&
                   text.at(i + 1) <= '9') {
            previous_digit = false;
        } else {
            return false;
        }
    }
    long long parsed = 0;
    const char* digits_end = std::next(digits.data(), static_cast<std::ptrdiff_t>(digits.size()));
    const auto result = std::from_chars(digits.data(), digits_end, parsed);
    if (result.ec != std::errc() || result.ptr != digits_end) {
        return false;
    }
    value = negative ? -parsed : parsed;
    return true;
}

std::string python_format_fixed(double value, int decimals) {
    if (std::isnan(value)) {
        return "nan";
    }
    if (std::isinf(value)) {
        return value > 0 ? "inf" : "-inf";
    }
    // double 최대(약 1.8e308)의 정수부 309자리 + 소수 decimals자리 + 부호·소수점이 들어가는 크기
    std::string out(static_cast<std::size_t>(decimals) + 512, '\0');
    char* const begin = out.data();
    const auto result =
        std::to_chars(begin, std::next(begin, static_cast<std::ptrdiff_t>(out.size())), value,
                      std::chars_format::fixed, decimals);
    if (result.ec != std::errc()) {
        throw std::overflow_error("숫자 표기 길이 초과");
    }
    out.resize(static_cast<std::size_t>(std::distance(begin, result.ptr)));
    return out;
}

std::optional<double> python_number(const nlohmann::json& value, double scale) {
    double number = 0.0;
    switch (value.type()) {
        case nlohmann::json::value_t::boolean:
            number = value.get<bool>() ? 1.0 : 0.0;
            break;
        case nlohmann::json::value_t::number_integer:
            number = static_cast<double>(value.get<std::int64_t>());
            break;
        case nlohmann::json::value_t::number_unsigned:
            number = static_cast<double>(value.get<std::uint64_t>());
            break;
        case nlohmann::json::value_t::number_float:
            number = value.get<double>();
            break;
        case nlohmann::json::value_t::string: {
            // float(str): 앞뒤 공백, 숫자 사이 밑줄 하나 허용 (16진 표기는 받지 않는다)
            static const std::regex kFloatPattern(
                R"([+-]?(?:\d(?:_?\d)*(?:\.(?:\d(?:_?\d)*)?)?|\.\d(?:_?\d)*)(?:[eE][+-]?\d(?:_?\d)*)?)");
            const std::string text =
                u32_to_utf8(python_strip(utf8_to_u32(value.get<std::string>())));
            if (!std::regex_match(text, kFloatPattern)) {
                return std::nullopt;  // inf/nan 문자열도 유한하지 않으므로 없음과 같다
            }
            std::string plain;
            for (const char c : text) {
                if (c != '_') {
                    plain += c;
                }
            }
            number = std::strtod(plain.c_str(), nullptr);
            break;
        }
        default:
            return std::nullopt;
    }
    number *= scale;
    if (!std::isfinite(number)) {
        return std::nullopt;
    }
    return number;
}

std::size_t utf8_length(const std::string& text) {
    std::size_t count = 0;
    for (const char c : text) {
        if ((static_cast<unsigned char>(c) & 0xc0U) != 0x80U) {
            ++count;
        }
    }
    return count;
}

}  // namespace argus::gateway
