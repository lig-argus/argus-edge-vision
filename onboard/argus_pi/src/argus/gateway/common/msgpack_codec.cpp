#include "argus/gateway/common/include/msgpack_codec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>

namespace argus::gateway {

namespace {

constexpr int kMaxDepth = 512;  // 원본 기본 재귀 한계보다 작지 않은 보호값

void put_be(Bytes& out, std::uint64_t value, int bytes) {
    for (int shift = (bytes - 1) * 8; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::uint8_t>((value >> static_cast<unsigned>(shift)) & 0xffU));
    }
}

void pack_length(Bytes& out, std::size_t size, std::uint8_t fix_base, std::size_t fix_limit,
                 std::uint8_t code8, std::uint8_t code16, std::uint8_t code32) {
    if (size < fix_limit) {
        out.push_back(static_cast<std::uint8_t>(fix_base | size));
    } else if (code8 != 0 && size <= 0xffU) {
        out.push_back(code8);
        put_be(out, size, 1);
    } else if (size <= 0xffffU) {
        out.push_back(code16);
        put_be(out, size, 2);
    } else if (size <= 0xffffffffULL) {
        out.push_back(code32);
        put_be(out, size, 4);
    } else {
        throw ValueError("msgpack 길이 초과");
    }
}

void pack_unsigned(Bytes& out, std::uint64_t value) {
    if (value < 0x80U) {
        out.push_back(static_cast<std::uint8_t>(value));
    } else if (value <= 0xffU) {
        out.push_back(0xcc);
        put_be(out, value, 1);
    } else if (value <= 0xffffU) {
        out.push_back(0xcd);
        put_be(out, value, 2);
    } else if (value <= 0xffffffffULL) {
        out.push_back(0xce);
        put_be(out, value, 4);
    } else {
        out.push_back(0xcf);
        put_be(out, value, 8);
    }
}

void pack_signed(Bytes& out, std::int64_t value) {
    if (value >= 0) {
        pack_unsigned(out, static_cast<std::uint64_t>(value));
    } else if (value >= -32) {
        out.push_back(static_cast<std::uint8_t>(value));
    } else if (value >= -128) {
        out.push_back(0xd0);
        put_be(out, static_cast<std::uint64_t>(value), 1);
    } else if (value >= -32768) {
        out.push_back(0xd1);
        put_be(out, static_cast<std::uint64_t>(value), 2);
    } else if (value >= std::numeric_limits<std::int32_t>::min()) {
        out.push_back(0xd2);
        put_be(out, static_cast<std::uint64_t>(value), 4);
    } else {
        out.push_back(0xd3);
        put_be(out, static_cast<std::uint64_t>(value), 8);
    }
}

void pack_string(Bytes& out, const std::string& text) {
    pack_length(out, text.size(), 0xa0, 32, 0xd9, 0xda, 0xdb);
    out.insert(out.end(), text.begin(), text.end());
}

void pack_value(Bytes& out, const Wire& value, int depth) {  // NOLINT(misc-no-recursion)
    if (depth > kMaxDepth) {
        throw ValueError("msgpack 중첩 깊이 초과");
    }
    switch (value.type()) {
        case Wire::value_t::null:
            out.push_back(0xc0);
            return;
        case Wire::value_t::boolean:
            out.push_back(value.get<bool>() ? 0xc3 : 0xc2);
            return;
        case Wire::value_t::number_unsigned:
            pack_unsigned(out, value.get<std::uint64_t>());
            return;
        case Wire::value_t::number_integer:
            pack_signed(out, value.get<std::int64_t>());
            return;
        case Wire::value_t::number_float: {
            const double number = value.get<double>();
            std::uint64_t bits = 0;
            std::memcpy(&bits, &number, sizeof(bits));
            out.push_back(0xcb);
            put_be(out, bits, 8);
            return;
        }
        case Wire::value_t::string:
            pack_string(out, value.get_ref<const std::string&>());
            return;
        case Wire::value_t::binary: {
            const auto& data = value.get_binary();
            pack_length(out, data.size(), 0, 0, 0xc4, 0xc5, 0xc6);
            out.insert(out.end(), data.begin(), data.end());
            return;
        }
        case Wire::value_t::array:
            pack_length(out, value.size(), 0x90, 16, 0, 0xdc, 0xdd);
            for (const auto& item : value) {
                pack_value(out, item, depth + 1);
            }
            return;
        case Wire::value_t::object:
            pack_length(out, value.size(), 0x80, 16, 0, 0xde, 0xdf);
            for (const auto& item : value.items()) {
                pack_string(out, item.key());
                pack_value(out, item.value(), depth + 1);
            }
            return;
        default:
            throw ValueError("msgpack 형식 미지원");
    }
}

bool valid_utf8(const std::string& text) {
    std::size_t i = 0;
    while (i < text.size()) {
        const auto lead = static_cast<unsigned char>(text.at(i));
        std::size_t length = 0;
        unsigned char low = 0x80;
        unsigned char high = 0xbf;
        if (lead < 0x80) {
            length = 1;
        } else if (lead >= 0xc2 && lead <= 0xdf) {
            length = 2;
        } else if (lead >= 0xe0 && lead <= 0xef) {
            length = 3;
            low = lead == 0xe0 ? 0xa0 : 0x80;
            high = lead == 0xed ? 0x9f : 0xbf;
        } else if (lead >= 0xf0 && lead <= 0xf4) {
            length = 4;
            low = lead == 0xf0 ? 0x90 : 0x80;
            high = lead == 0xf4 ? 0x8f : 0xbf;
        } else {
            return false;
        }
        if (i + length > text.size()) {
            return false;
        }
        for (std::size_t k = 1; k < length; ++k) {
            const auto next = static_cast<unsigned char>(text.at(i + k));
            if (next < (k == 1 ? low : 0x80) || next > (k == 1 ? high : 0xbf)) {
                return false;
            }
        }
        i += length;
    }
    return true;
}

class Reader {
public:
    Reader(const Bytes& data, bool reject_duplicates) : data_(&data), reject_(reject_duplicates) {}

    Wire value(int depth) {  // NOLINT(misc-no-recursion)
        if (depth > kMaxDepth) {
            throw ValueError("msgpack 중첩 깊이 초과");
        }
        const std::uint8_t code = byte();
        if (code <= 0x7f) {
            return code;
        }
        if (code >= 0xe0) {
            return static_cast<std::int64_t>(static_cast<std::int8_t>(code));
        }
        if ((code & 0xe0U) == 0xa0U) {
            return text(code & 0x1fU);
        }
        if ((code & 0xf0U) == 0x90U) {
            return array(code & 0x0fU, depth);
        }
        if ((code & 0xf0U) == 0x80U) {
            return map(code & 0x0fU, depth);
        }
        switch (code) {
            case 0xc0:
                return nullptr;
            case 0xc2:
                return false;
            case 0xc3:
                return true;
            case 0xc4:
                return binary(read_be(1));
            case 0xc5:
                return binary(read_be(2));
            case 0xc6:
                return binary(read_be(4));
            case 0xca: {
                const auto bits = static_cast<std::uint32_t>(read_be(4));
                float number = 0;
                std::memcpy(&number, &bits, sizeof(number));
                return static_cast<double>(number);
            }
            case 0xcb: {
                const std::uint64_t bits = read_be(8);
                double number = 0;
                std::memcpy(&number, &bits, sizeof(number));
                return number;
            }
            case 0xcc:
                return read_be(1);
            case 0xcd:
                return read_be(2);
            case 0xce:
                return read_be(4);
            case 0xcf:
                return read_be(8);
            case 0xd0:
                return static_cast<std::int64_t>(static_cast<std::int8_t>(read_be(1)));
            case 0xd1:
                return static_cast<std::int64_t>(static_cast<std::int16_t>(read_be(2)));
            case 0xd2:
                return static_cast<std::int64_t>(static_cast<std::int32_t>(read_be(4)));
            case 0xd3:
                return static_cast<std::int64_t>(read_be(8));
            case 0xd9:
                return text(read_be(1));
            case 0xda:
                return text(read_be(2));
            case 0xdb:
                return text(read_be(4));
            case 0xdc:
                return array(read_be(2), depth);
            case 0xdd:
                return array(read_be(4), depth);
            case 0xde:
                return map(read_be(2), depth);
            case 0xdf:
                return map(read_be(4), depth);
            default:
                throw ValueError("msgpack 확장 형식 미지원");
        }
    }

    bool done() const { return position_ == data_->size(); }

private:
    std::uint8_t byte() {
        if (position_ >= data_->size()) {
            throw ValueError("Unpack failed: incomplete input");
        }
        return data_->at(position_++);
    }
    std::uint64_t read_be(int count) {
        std::uint64_t value = 0;
        for (int i = 0; i < count; ++i) {
            value = (value << 8U) | byte();
        }
        return value;
    }
    std::string raw(std::uint64_t size) {
        if (size > data_->size() - position_) {
            throw ValueError("Unpack failed: incomplete input");
        }
        std::string out(std::next(data_->begin(), static_cast<std::ptrdiff_t>(position_)),
                        std::next(data_->begin(), static_cast<std::ptrdiff_t>(position_ + size)));
        position_ += size;
        return out;
    }
    Wire text(std::uint64_t size) {
        std::string out = raw(size);
        if (!valid_utf8(out)) {
            throw ValueError("'utf-8' codec can't decode bytes");
        }
        return out;
    }
    Wire binary(std::uint64_t size) {
        const std::string out = raw(size);
        return Wire::binary(std::vector<std::uint8_t>(out.begin(), out.end()));
    }
    Wire array(std::uint64_t size, int depth) {  // NOLINT(misc-no-recursion)
        Wire out = Wire::array();
        for (std::uint64_t i = 0; i < size; ++i) {
            out.push_back(value(depth + 1));
        }
        return out;
    }
    Wire map(std::uint64_t size, int depth) {  // NOLINT(misc-no-recursion)
        Wire out = Wire::object();
        for (std::uint64_t i = 0; i < size; ++i) {
            const Wire key = value(depth + 1);
            if (!key.is_string()) {
                throw ValueError("map key must be str");
            }
            Wire item = value(depth + 1);
            const auto& name = key.get_ref<const std::string&>();
            if (out.contains(name)) {
                if (reject_) {
                    throw ValueError("duplicate_tracking_field");
                }
                out.at(name) = std::move(item);  // 원본 dict: 나중 값, 처음 자리
            } else {
                out.emplace(name, std::move(item));
            }
        }
        return out;
    }

    const Bytes* data_;
    bool reject_;
    std::size_t position_{0};
};

bool numeric(const Wire& value) { return value.is_number() || value.is_boolean(); }

// Python str.isprintable()이 거짓인 문자 (제어·서식·구분 문자 중 실제로 나올 만한 것).
// 전체 유니코드 표는 두지 않는다. 한글·일반 기호는 출력 가능으로 본다.
bool python_unprintable(char32_t code) {
    if (code < 0x20 || (code >= 0x7f && code <= 0xa0) || code == 0xad) {
        return true;
    }
    if (code == 0x1680 || (code >= 0x2000 && code <= 0x200f) ||
        (code >= 0x2028 && code <= 0x202f) || (code >= 0x205f && code <= 0x2064) ||
        code == 0x3000 || code == 0xfeff) {
        return true;
    }
    return (code >= 0xd800 && code <= 0xdfff) || (code >= 0xe000 && code <= 0xf8ff) ||
           code == 0xfffe || code == 0xffff || code > 0x10ffff;
}

std::string hex_escape(const char* prefix, std::uint32_t code, int digits) {
    static constexpr std::array<char, 16> kDigits{'0', '1', '2', '3', '4', '5', '6', '7',
                                                  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    std::string out(prefix);
    for (int shift = (digits - 1) * 4; shift >= 0; shift -= 4) {
        out += kDigits.at((code >> static_cast<unsigned>(shift)) & 0xfU);
    }
    return out;
}

// Python repr(str) / repr(bytes)의 따옴표: 작은따옴표만 있으면 큰따옴표, 아니면 작은따옴표
char repr_quote(const std::string& text) {
    const bool single = text.find('\'') != std::string::npos;
    const bool doubled = text.find('"') != std::string::npos;
    return single && !doubled ? '"' : '\'';
}

std::string repr_text(const std::string& text) {
    const char quote = repr_quote(text);
    std::string out(1, quote);
    for (const char32_t code : utf8_to_u32(text)) {
        if (code == U'\\' || code == static_cast<char32_t>(quote)) {
            out += '\\';
            out += static_cast<char>(code);
        } else if (code == U'\n') {
            out += "\\n";
        } else if (code == U'\r') {
            out += "\\r";
        } else if (code == U'\t') {
            out += "\\t";
        } else if (python_unprintable(code)) {
            if (code <= 0xff) {
                out += hex_escape("\\x", code, 2);
            } else if (code <= 0xffff) {
                out += hex_escape("\\u", code, 4);
            } else {
                out += hex_escape("\\U", code, 8);
            }
        } else {
            out += u32_to_utf8(std::u32string(1, code));
        }
    }
    out += quote;
    return out;
}

std::string repr_bytes(const Wire::binary_t& data) {
    const std::string text(data.begin(), data.end());
    const char quote = repr_quote(text);
    std::string out = "b";
    out += quote;
    for (const std::uint8_t byte : data) {
        if (byte == '\\' || byte == static_cast<std::uint8_t>(quote)) {
            out += '\\';
            out += static_cast<char>(byte);
        } else if (byte == '\n') {
            out += "\\n";
        } else if (byte == '\r') {
            out += "\\r";
        } else if (byte == '\t') {
            out += "\\t";
        } else if (byte < 0x20 || byte >= 0x7f) {
            out += hex_escape("\\x", byte, 2);
        } else {
            out += static_cast<char>(byte);
        }
    }
    out += quote;
    return out;
}

void json_string(std::string& out, const std::string& text, bool ensure_ascii) {
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
        const bool escape = code < 0x20 || (ensure_ascii && code > 0x7e);
        if (!escape) {
            out += u32_to_utf8(std::u32string(1, code));
        } else if (code > 0xffff) {
            const char32_t value = code - 0x10000;
            out += hex_escape("\\u", 0xd800 + (value >> 10U), 4);
            out += hex_escape("\\u", 0xdc00 + (value & 0x3ffU), 4);
        } else {
            out += hex_escape("\\u", code, 4);
        }
    }
    out += '"';
}

// NOLINTNEXTLINE(misc-no-recursion): 중첩 값을 재귀로 쓴다 (깊이는 입력 구조만큼)
void json_value(std::string& out, const Wire& value, bool ensure_ascii, bool compact) {
    const char* item = compact ? "," : ", ";
    const char* key = compact ? ":" : ": ";
    switch (value.type()) {
        case Wire::value_t::null:
            out += "null";
            return;
        case Wire::value_t::boolean:
            out += value.get<bool>() ? "true" : "false";
            return;
        case Wire::value_t::number_float: {
            const double number = value.get<double>();
            if (!std::isfinite(number)) {
                throw ValueError("Out of range float values are not JSON compliant: " +
                                 python_float_repr(number));
            }
            out += python_float_repr(number);
            return;
        }
        case Wire::value_t::string:
            json_string(out, value.get_ref<const std::string&>(), ensure_ascii);
            return;
        case Wire::value_t::array: {
            out += '[';
            for (std::size_t i = 0; i < value.size(); ++i) {
                out += i == 0 ? "" : item;
                json_value(out, value.at(i), ensure_ascii, compact);
            }
            out += ']';
            return;
        }
        case Wire::value_t::object: {
            out += '{';
            bool first = true;
            for (const auto& entry : value.items()) {
                out += first ? "" : item;
                first = false;
                json_string(out, entry.key(), ensure_ascii);
                out += key;
                json_value(out, entry.value(), ensure_ascii, compact);
            }
            out += '}';
            return;
        }
        case Wire::value_t::binary:
            throw std::runtime_error("TypeError: Object of type bytes is not JSON serializable");
        default:
            out += value.dump();  // 정수
            return;
    }
}

}  // namespace

Bytes msgpack_pack(const Wire& value) {
    Bytes out;
    pack_value(out, value, 0);
    return out;
}

Wire msgpack_unpack(const Bytes& data, bool reject_duplicate_keys) {
    Reader reader(data, reject_duplicate_keys);
    Wire value = reader.value(0);
    if (!reader.done()) {
        throw ValueError("Unpack failed: extra data");
    }
    return value;
}

bool python_equal(const Wire& left, const Wire& right) {  // NOLINT(misc-no-recursion)
    if (numeric(left) && numeric(right)) {
        const bool left_float = left.is_number_float();
        const bool right_float = right.is_number_float();
        if (!left_float && !right_float) {
            const auto as_pair = [](const Wire& value) -> std::pair<bool, std::uint64_t> {
                if (value.is_boolean()) {
                    return {false, value.get<bool>() ? 1U : 0U};
                }
                if (value.is_number_unsigned()) {
                    return {false, value.get<std::uint64_t>()};
                }
                const std::int64_t number = value.get<std::int64_t>();
                return {number < 0, number < 0 ? static_cast<std::uint64_t>(-(number + 1)) + 1
                                               : static_cast<std::uint64_t>(number)};
            };
            return as_pair(left) == as_pair(right);
        }
        const auto as_double = [](const Wire& value) {
            if (value.is_boolean()) {
                return value.get<bool>() ? 1.0 : 0.0;
            }
            return value.get<double>();
        };
        return as_double(left) == as_double(right);
    }
    if (left.type() != right.type()) {
        return false;
    }
    if (left.is_array()) {
        if (left.size() != right.size()) {
            return false;
        }
        for (std::size_t i = 0; i < left.size(); ++i) {
            if (!python_equal(left.at(i), right.at(i))) {
                return false;
            }
        }
        return true;
    }
    if (left.is_object()) {
        if (left.size() != right.size()) {
            return false;
        }
        const auto items = left.items();
        // 원본 dict ==처럼 중첩 값을 재귀 비교한다 (깊이는 unpack의 kMaxDepth로 제한).
        // NOLINTNEXTLINE(misc-no-recursion)
        return std::all_of(items.begin(), items.end(), [&right](const auto& item) {
            return right.contains(item.key()) && python_equal(item.value(), right.at(item.key()));
        });
    }
    return left == right;
}

std::string python_str_value(const Wire& value) {
    if (value.is_string()) {
        return value.get<std::string>();
    }
    if (value.is_null()) {
        return "None";
    }
    if (value.is_boolean()) {
        return value.get<bool>() ? "True" : "False";
    }
    return python_repr_value(value);
}

// 목록·사전은 원소마다 재귀로 쓴다 (깊이는 값의 중첩만큼)
std::string python_repr_value(const Wire& value) {  // NOLINT(misc-no-recursion)
    switch (value.type()) {
        case Wire::value_t::null:
            return "None";
        case Wire::value_t::boolean:
            return value.get<bool>() ? "True" : "False";
        case Wire::value_t::number_float:
            return python_float_repr(value.get<double>());
        case Wire::value_t::string:
            return repr_text(value.get_ref<const std::string&>());
        case Wire::value_t::binary:
            return repr_bytes(value.get_binary());
        case Wire::value_t::array: {
            std::string out = "[";
            for (std::size_t i = 0; i < value.size(); ++i) {
                out += (i == 0 ? "" : ", ") + python_repr_value(value.at(i));
            }
            return out + "]";
        }
        case Wire::value_t::object: {
            std::string out = "{";
            bool first = true;
            for (const auto& item : value.items()) {
                out += (first ? "" : ", ") + repr_text(item.key()) + ": " +
                       python_repr_value(item.value());
                first = false;
            }
            return out + "}";
        }
        default:
            return value.dump();  // 정수
    }
}

bool wire_truthy(const Wire& value) {
    switch (value.type()) {
        case Wire::value_t::null:
            return false;
        case Wire::value_t::boolean:
            return value.get<bool>();
        case Wire::value_t::number_integer:
            return value.get<std::int64_t>() != 0;
        case Wire::value_t::number_unsigned:
            return value.get<std::uint64_t>() != 0;
        case Wire::value_t::number_float:
            return value.get<double>() != 0.0;
        case Wire::value_t::binary:
            return !value.get_binary().empty();
        case Wire::value_t::string:
            return !value.get_ref<const std::string&>().empty();  // empty()는 문자열에 항상 false
        default:
            return !value.empty();
    }
}

Wire wire_get(const Wire& object, const std::string& key) {
    if (!object.is_object()) {
        return nullptr;
    }
    const auto found = object.find(key);
    return found == object.end() ? Wire() : *found;
}

}  // namespace argus::gateway

namespace argus::gateway {
namespace {

// NOLINTNEXTLINE(misc-no-recursion): 중첩 값을 재귀로 쓴다 (깊이는 입력 구조만큼)
void json_indented(std::string& out, const Wire& value, int indent, int level, bool ensure_ascii) {
    const std::string inner(static_cast<std::size_t>(indent * (level + 1)), ' ');
    const std::string outer(static_cast<std::size_t>(indent * level), ' ');
    if (value.is_array() && !value.empty()) {
        out += "[\n";
        for (std::size_t i = 0; i < value.size(); ++i) {
            out += (i == 0 ? "" : ",\n") + inner;
            json_indented(out, value.at(i), indent, level + 1, ensure_ascii);
        }
        out += "\n" + outer + "]";
        return;
    }
    if (value.is_object() && !value.empty()) {
        out += "{\n";
        bool first = true;
        for (const auto& entry : value.items()) {
            out += (first ? "" : ",\n") + inner;
            first = false;
            json_string(out, entry.key(), ensure_ascii);
            out += ": ";
            json_indented(out, entry.value(), indent, level + 1, ensure_ascii);
        }
        out += "\n" + outer + "}";
        return;
    }
    json_value(out, value, ensure_ascii, false);
}

}  // namespace
}  // namespace argus::gateway

namespace argus::gateway {

std::string python_json_dumps(const Wire& value, bool ensure_ascii, bool compact) {
    std::string out;
    json_value(out, value, ensure_ascii, compact);
    return out;
}

}  // namespace argus::gateway

namespace argus::gateway {

std::string python_json_dumps_indent(const Wire& value, int indent, bool ensure_ascii) {
    std::string out;
    json_indented(out, value, indent, 0, ensure_ascii);
    return out;
}

}  // namespace argus::gateway
