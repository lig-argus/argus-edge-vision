#include "argus/gateway/events/include/event_parser.hpp"

#include <charconv>
#include <cmath>
#include <cstring>
#include <iterator>
#include <utility>

namespace argus::gateway {

namespace {

using Json = nlohmann::ordered_json;

// Python repr(str): 작은따옴표 기본, 작은따옴표만 들어 있으면 큰따옴표
std::string python_repr_text(const std::string& text) {
    const bool single = text.find('\'') != std::string::npos;
    const bool doubled = text.find('"') != std::string::npos;
    const char quote = single && !doubled ? '"' : '\'';
    std::string out(1, quote);
    for (const char c : text) {
        if (c == '\\' || c == quote) {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else if (c == '\t') {
            out += "\\t";
        } else {
            out += c;
        }
    }
    out += quote;
    return out;
}

[[noreturn]] void key_error(const std::string& key) {
    throw EventDecodeError(python_repr_text(key));
}

const Json& at_key(const Json& object, const std::string& key) {
    if (!object.is_object()) {
        throw EventDecodeError("TypeError: 객체가 아닙니다");
    }
    const auto found = object.find(key);
    if (found == object.end()) {
        key_error(key);
    }
    return *found;
}

std::string text_of(const Json& value) {
    if (!value.is_string()) {
        throw EventDecodeError("TypeError: 문자열이 아닙니다");
    }
    return value.get<std::string>();
}

long long int_key(const std::string& key) {
    long long value = 0;
    if (!python_parse_int(key, value)) {
        throw EventDecodeError("invalid literal for int() with base 10: " + python_repr_text(key));
    }
    return value;
}

std::size_t base_type_size(const std::string& type) {
    if (type == "uint8_t" || type == "int8_t") {
        return 1;
    }
    if (type == "uint16_t" || type == "int16_t") {
        return 2;
    }
    if (type == "uint32_t" || type == "int32_t" || type == "float") {
        return 4;
    }
    if (type == "uint64_t" || type == "int64_t") {
        return 8;
    }
    return 0;
}

bool is_base_type(const std::string& type) { return base_type_size(type) != 0; }

// Python 정수/실수 값 하나
struct Number {
    enum class Kind : std::uint8_t { Signed, Unsigned, Float } kind{Kind::Signed};
    std::int64_t signed_value{0};
    std::uint64_t unsigned_value{0};
    double float_value{0.0};

    std::string str() const {
        switch (kind) {
            case Kind::Signed:
                return std::to_string(signed_value);
            case Kind::Unsigned:
                return std::to_string(unsigned_value);
            case Kind::Float:
            default:
                return python_float_repr(float_value);
        }
    }
    double as_double() const {
        switch (kind) {
            case Kind::Signed:
                return static_cast<double>(signed_value);
            case Kind::Unsigned:
                return static_cast<double>(unsigned_value);
            case Kind::Float:
            default:
                return float_value;
        }
    }
    bool equals_integer(long long other) const {
        switch (kind) {
            case Kind::Signed:
                return signed_value == other;
            case Kind::Unsigned:
                return other >= 0 && unsigned_value == static_cast<std::uint64_t>(other);
            case Kind::Float:
            default:
                return float_value == static_cast<double>(other);
        }
    }
};

Number unpack(const std::string& type, const Bytes& data, std::size_t offset) {
    std::uint64_t raw = 0;
    const std::size_t size = base_type_size(type);
    for (std::size_t i = 0; i < size; ++i) {
        raw |= static_cast<std::uint64_t>(data.at(offset + i)) << (8 * i);
    }
    Number number;
    if (type == "float") {
        float value = 0.0F;
        const auto bits = static_cast<std::uint32_t>(raw);
        std::memcpy(&value, &bits, sizeof(value));
        number.kind = Number::Kind::Float;
        number.float_value = static_cast<double>(value);
    } else if (type.at(0) == 'u') {
        number.kind = Number::Kind::Unsigned;
        number.unsigned_value = raw;
    } else {
        number.kind = Number::Kind::Signed;
        const auto shift = static_cast<unsigned>(64 - (size * 8));
        number.signed_value = static_cast<std::int64_t>(raw << shift) >> shift;
    }
    return number;
}

struct Argument {
    Number value;
    const Json* enum_definition{nullptr};
};

// 표시 값: 열거형이면 문자열, 아니면 숫자
struct Display {
    bool is_text{false};
    std::string text;
    Number number;
};

Display display_value(const Argument& argument) {
    Display display;
    if (argument.enum_definition == nullptr) {
        display.number = argument.value;
        return display;
    }
    const Json& definition = *argument.enum_definition;
    display.is_text = true;
    const auto bitfield = definition.find("is_bitfield");
    const bool is_bitfield =
        bitfield != definition.end() && python_truthy(nlohmann::json::parse(bitfield->dump()));
    if (is_bitfield) {
        const std::size_t size = base_type_size(text_of(at_key(definition, "type")));
        if (size == 0) {
            key_error(text_of(at_key(definition, "type")));
        }
        if (argument.value.kind == Number::Kind::Float) {
            throw EventDecodeError("unsupported operand type(s) for &: 'float' and 'int'");
        }
        const std::uint64_t bits = argument.value.kind == Number::Kind::Signed
                                       ? static_cast<std::uint64_t>(argument.value.signed_value)
                                       : argument.value.unsigned_value;
        const auto separator_found = definition.find("separator");
        const std::string separator =
            separator_found == definition.end() ? std::string("|") : text_of(*separator_found);
        bool had_bit = false;
        for (std::size_t i = 0; i < size * 8; ++i) {
            const std::uint64_t bit = std::uint64_t{1} << i;
            if ((bits & bit) == 0) {
                continue;
            }
            if (had_bit) {
                display.text += separator;
            }
            bool found = false;
            for (const auto& entry : at_key(definition, "entries").items()) {
                if (static_cast<std::uint64_t>(int_key(entry.key())) == bit &&
                    int_key(entry.key()) >= 0) {
                    display.text += text_of(at_key(entry.value(), "description"));
                    found = true;
                    break;
                }
            }
            if (!found) {
                display.text += "(unknown: " + argument.value.str() + ")";
            }
            had_bit = true;
        }
        return display;
    }
    for (const auto& entry : at_key(definition, "entries").items()) {
        if (argument.value.equals_integer(int_key(entry.key()))) {
            display.text = text_of(at_key(entry.value(), "description"));
            return display;
        }
    }
    display.text = "(unknown: " + argument.value.str() + ")";
    return display;
}

std::string format_fixed(double value, int decimals) {
    try {
        return python_format_fixed(value, decimals);
    } catch (const std::overflow_error&) {
        throw EventDecodeError("OverflowError: 숫자 표기 길이 초과");
    }
}

bool is_ascii_digit(char32_t c) { return c >= U'0' && c <= U'9'; }

// ^(\d+)(?::(?:\.(\d+))?)?(m|m_v|m/s|m\^2|C)?$ ($는 끝의 \n 하나 앞에서도 맞는다)
bool parse_argument_spec(std::u32string spec, std::string& index_digits,
                         std::optional<int>& decimals, std::string& unit) {
    const auto try_match = [&](const std::u32string& text) {
        std::size_t pos = 0;
        while (pos < text.size() && is_ascii_digit(text.at(pos))) {
            ++pos;
        }
        if (pos == 0) {
            return false;
        }
        index_digits = u32_to_utf8(text.substr(0, pos));
        decimals.reset();
        if (pos < text.size() && text.at(pos) == U':') {
            ++pos;
            if (pos < text.size() && text.at(pos) == U'.') {
                const std::size_t start = pos + 1;
                std::size_t end = start;
                while (end < text.size() && is_ascii_digit(text.at(end))) {
                    ++end;
                }
                if (end == start) {
                    return false;
                }
                const std::string digits = u32_to_utf8(text.substr(start, end - start));
                int parsed = 0;
                const auto result = std::from_chars(
                    digits.data(),
                    std::next(digits.data(), static_cast<std::ptrdiff_t>(digits.size())), parsed);
                decimals = result.ec == std::errc() && parsed <= 1000 ? parsed : 1000;
                pos = end;
            }
        }
        const std::string rest = u32_to_utf8(text.substr(pos));
        if (!rest.empty() && rest != "m" && rest != "m_v" && rest != "m/s" && rest != "m^2" &&
            rest != "C") {
            return false;
        }
        unit = rest;
        return true;
    };
    if (try_match(spec)) {
        return true;
    }
    if (!spec.empty() && spec.back() == U'\n') {
        spec.pop_back();
        return try_match(spec);
    }
    return false;
}

struct TagMatch {
    std::u32string name;
    std::optional<std::u32string> attribute;
    std::size_t content_start{0};
};

// ^<([a-z]+)(?: ([a-z]+)="([^"]*)")?>(.)  (DOTALL)
std::optional<TagMatch> match_tag(const std::u32string& message, std::size_t at) {
    const auto lower = [](char32_t c) { return c >= U'a' && c <= U'z'; };
    std::size_t pos = at + 1;
    const std::size_t name_begin = pos;
    while (pos < message.size() && lower(message.at(pos))) {
        ++pos;
    }
    if (pos == name_begin) {
        return std::nullopt;
    }
    TagMatch match;
    match.name = message.substr(name_begin, pos - name_begin);
    if (pos < message.size() && message.at(pos) == U' ') {
        std::size_t cursor = pos + 1;
        const std::size_t attribute_begin = cursor;
        while (cursor < message.size() && lower(message.at(cursor))) {
            ++cursor;
        }
        if (cursor > attribute_begin && cursor + 1 < message.size() && message.at(cursor) == U'=' &&
            message.at(cursor + 1) == U'"') {
            const std::size_t value_begin = cursor + 2;
            const std::size_t value_end = message.find(U'"', value_begin);
            if (value_end != std::u32string::npos) {
                match.attribute = message.substr(value_begin, value_end - value_begin);
                pos = value_end + 1;
            }
        }
    }
    if (pos >= message.size() || message.at(pos) != U'>' || pos + 1 >= message.size()) {
        return std::nullopt;
    }
    match.content_start = pos + 1;
    return match;
}

std::string process_message(std::u32string message, const std::vector<Argument>& arguments,
                            const std::string& profile) {
    std::size_t i = 0;
    while (i < message.size()) {
        if (message.at(i) == U'\\') {
            message.erase(i, 1);
            i += 1;
            continue;
        }
        if (message.at(i) == U'<') {
            const std::optional<TagMatch> tag = match_tag(message, i);
            if (!tag) {
                i += 1;
                continue;
            }
            const std::size_t end_tag = message.find(U"</" + tag->name + U">", tag->content_start);
            if (end_tag == std::u32string::npos) {
                i += 1;
                continue;
            }
            std::u32string content =
                message.substr(tag->content_start, end_tag - tag->content_start);
            std::size_t skip = 0;
            if (tag->name == U"profile") {
                const std::string wanted =
                    tag->attribute ? u32_to_utf8(*tag->attribute) : std::string();
                const bool remove = !wanted.empty() && wanted.at(0) == '!'
                                        ? profile == wanted.substr(1)
                                        : profile != wanted;
                if (remove) {
                    content.clear();
                }
            } else if (tag->name == U"a" || tag->name == U"param") {
                skip = content.size();  // 기본 형식 지정기는 내용을 그대로 둔다
            } else {
                content.clear();
            }
            message.replace(i, end_tag + tag->name.size() + 3 - i, content);
            i += skip;
            continue;
        }
        if (message.at(i) == U'{') {
            const std::size_t close = message.find(U'}', i);
            if (close == std::u32string::npos) {
                i += 1;
                continue;
            }
            std::string index_digits;
            std::optional<int> decimals;
            std::string unit;
            if (!parse_argument_spec(message.substr(i + 1, close - i - 1), index_digits, decimals,
                                     unit)) {
                i += 1;
                continue;
            }
            long long index = 0;
            const auto parsed = std::from_chars(
                index_digits.data(),
                std::next(index_digits.data(), static_cast<std::ptrdiff_t>(index_digits.size())),
                index);
            index = parsed.ec == std::errc() ? index - 1 : -1;
            if (index < 0 || index >= static_cast<long long>(arguments.size())) {
                i += 1;
                continue;
            }
            const Display display = display_value(arguments.at(static_cast<std::size_t>(index)));
            std::string value;
            if (decimals) {
                if (display.is_text) {
                    throw EventDecodeError("Unknown format code 'f' for object of type 'str'");
                }
                value = format_fixed(display.number.as_double(), *decimals);
            } else {
                value = display.is_text ? display.text : display.number.str();
            }
            if (!unit.empty()) {
                value += " " + (unit == "m_v" ? std::string("m") : unit);
            }
            const std::u32string wide = utf8_to_u32(value);
            message.replace(i, close + 1 - i, wide);
            i += wide.size();
            continue;
        }
        i += 1;
    }
    return u32_to_utf8(python_strip(message));
}

}  // namespace

void EventParser::load_definitions(nlohmann::ordered_json definitions) {
    components_.clear();
    enums_.clear();
    definitions_ = std::move(definitions);
    const auto version_found = definitions_.find("version");
    const Json version = version_found == definitions_.end() ? Json(0) : *version_found;
    if (!version.is_number()) {
        throw EventDecodeError("TypeError: version");
    }
    if (version.get<double>() < 1) {
        const std::string shown =
            version.is_number_float() ? python_float_repr(version.get<double>()) : version.dump();
        throw EventDecodeError("Unexpected version " + shown);
    }
    const auto components = definitions_.find("components");
    if (components == definitions_.end()) {
        return;
    }
    for (const auto& component : components->items()) {
        const long long component_id = int_key(component.key());
        Component index;
        const Json& value = component.value();
        const auto groups = value.find("event_groups");
        if (groups != value.end()) {
            for (const auto& group : groups->items()) {
                Group entry;
                entry.name = group.key();
                const auto events = group.value().find("events");
                if (events != group.value().end()) {
                    for (const auto& event : events->items()) {
                        entry.events[int_key(event.key())] = &event.value();
                    }
                }
                index.groups.push_back(std::move(entry));
            }
        }
        components_[component_id] = std::move(index);
    }
    // 원본은 두 번째 반복에서 namespace와 열거형을 모은다 (namespace 없으면 KeyError)
    for (const auto& component : components->items()) {
        const std::string name_space = text_of(at_key(component.value(), "namespace"));
        components_[int_key(component.key())].name_space = name_space;
        const auto enums = component.value().find("enums");
        if (enums != component.value().end()) {
            for (const auto& item : enums->items()) {
                enums_[name_space + "::" + item.key()] = &item.value();
            }
        }
    }
}

std::optional<std::string> EventParser::name(std::uint32_t event_id) const {
    const long long component_id = (event_id >> 24) & 0xff;
    const long long sub_id = event_id & 0xffffff;
    const auto component = components_.find(component_id);
    if (component == components_.end()) {
        return std::nullopt;
    }
    for (const Group& group : component->second.groups) {
        const auto event_found = group.events.find(sub_id);
        if (event_found == group.events.end()) {
            continue;
        }
        const auto found = event_found->second->find("name");
        if (found != event_found->second->end() && found->is_string()) {
            return found->get<std::string>();
        }
        return std::nullopt;
    }
    return std::nullopt;
}

std::optional<ParsedEventText> EventParser::parse(std::uint32_t event_id,
                                                  const Bytes& arguments) const {
    const long long component_id = (event_id >> 24) & 0xff;
    const long long sub_id = event_id & 0xffffff;
    const auto component = components_.find(component_id);
    if (component == components_.end()) {
        return std::nullopt;
    }
    for (const Group& group : component->second.groups) {
        const auto event_found = group.events.find(sub_id);
        if (event_found == group.events.end()) {
            continue;
        }
        const Json& event = *event_found->second;
        std::vector<Argument> values;
        std::size_t offset = 0;
        const auto argument_list = event.find("arguments");
        if (argument_list != event.end()) {
            for (const Json& argument : *argument_list) {
                const std::string type = text_of(at_key(argument, "type"));
                Argument value;
                std::string base = type;
                if (!is_base_type(type)) {
                    const std::string enum_type = type.find("::") != std::string::npos
                                                      ? type
                                                      : component->second.name_space + "::" + type;
                    const auto found = enums_.find(enum_type);
                    if (found == enums_.end()) {
                        key_error(enum_type);
                    }
                    value.enum_definition = found->second;
                    base = text_of(at_key(*found->second, "type"));
                }
                const std::size_t size = base_type_size(base);
                if (size == 0) {
                    key_error(base);
                }
                if (offset + size > arguments.size()) {
                    throw EventDecodeError("unpack requires a buffer of " + std::to_string(size) +
                                           " bytes");
                }
                value.value = unpack(base, arguments, offset);
                values.push_back(value);
                offset += size;
            }
        }
        ParsedEventText text;
        text.message =
            process_message(utf8_to_u32(text_of(at_key(event, "message"))), values, profile_);
        text.group = group.name;
        const auto description = event.find("description");
        if (description != event.end()) {
            text.description =
                process_message(utf8_to_u32(text_of(*description)), values, profile_);
        }
        return text;
    }
    return std::nullopt;
}

}  // namespace argus::gateway
