// 버스 코덱의 기본 자료형 검사와 필드 쓰기를 공유한다. 각 schema 검증은 코덱이 맡는다.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <msgpack.hpp>
#include <optional>
#include <stdexcept>
#include <string>

namespace follow {

inline std::string msgpack_read_string(const msgpack::object& value, std::size_t limit,
                                       const char* error = "expected bounded string") {
    if (value.type != msgpack::type::STR || value.via.str.size > limit) {
        throw std::runtime_error(error);
    }
    return {value.via.str.ptr, value.via.str.size};
}

inline std::uint64_t msgpack_read_u64(const msgpack::object& value,
                                      const char* error = "expected non-negative integer") {
    if (value.type != msgpack::type::POSITIVE_INTEGER) {
        throw std::runtime_error(error);
    }
    return value.as<std::uint64_t>();
}

inline std::int64_t msgpack_read_i64(const msgpack::object& value,
                                     const char* error = "expected integer",
                                     const char* overflow = "integer exceeds signed range") {
    if (value.type == msgpack::type::POSITIVE_INTEGER) {
        const auto number = value.as<std::uint64_t>();
        if (number > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
            throw std::runtime_error(overflow);
        }
        return static_cast<std::int64_t>(number);
    }
    if (value.type == msgpack::type::NEGATIVE_INTEGER) {
        return value.as<std::int64_t>();
    }
    throw std::runtime_error(error);
}

inline bool msgpack_read_bool(const msgpack::object& value,
                              const char* error = "expected boolean") {
    if (value.type != msgpack::type::BOOLEAN) {
        throw std::runtime_error(error);
    }
    return value.as<bool>();
}

inline double msgpack_read_number(const msgpack::object& value,
                                  const char* error = "expected numeric value") {
    if (value.type != msgpack::type::FLOAT32 && value.type != msgpack::type::FLOAT64 &&
        value.type != msgpack::type::POSITIVE_INTEGER &&
        value.type != msgpack::type::NEGATIVE_INTEGER) {
        throw std::runtime_error(error);
    }
    return value.as<double>();
}

inline double msgpack_read_finite_number(const msgpack::object& value,
                                         const char* error = "expected number",
                                         const char* nonfinite = "number must be finite") {
    const auto number = msgpack_read_number(value, error);
    if (!std::isfinite(number)) {
        throw std::runtime_error(nonfinite);
    }
    return number;
}

inline std::optional<std::int64_t> msgpack_read_optional_i64(const msgpack::object& value) {
    if (value.type == msgpack::type::NIL) {
        return std::nullopt;
    }
    return msgpack_read_i64(value);
}

template <typename T>
void msgpack_pack_field(msgpack::packer<msgpack::sbuffer>& packer, const char* key,
                        const T& value) {
    packer.pack(std::string(key));
    packer.pack(value);
}

template <typename T>
void msgpack_pack_optional(msgpack::packer<msgpack::sbuffer>& packer, const char* key,
                           const std::optional<T>& value) {
    packer.pack(std::string(key));
    if (value) {
        packer.pack(*value);
    } else {
        packer.pack_nil();
    }
}

}  // namespace follow
