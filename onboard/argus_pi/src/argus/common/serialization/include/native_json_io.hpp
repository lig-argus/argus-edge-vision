// main 내부 파일·기존 MessagePack 봉투·host monotonic 시각의 공통 입출력.
#pragma once
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
namespace follow {
using NativeJson = nlohmann::json;
inline std::int64_t native_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
inline NativeJson read_native_json(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot read " + path.string());
    }
    NativeJson out;
    file >> out;
    return out;
}
inline void write_native_json(const std::filesystem::path& path, const NativeJson& value) {
    auto next = path.string() + ".next";
    {
        std::ofstream file(next);
        file << value.dump(2) << '\n';
        if (!file) {
            throw std::runtime_error("cannot write " + next);
        }
    }
    std::filesystem::rename(next, path);
}
inline bool native_env(const char* name) {
    const char* value = std::getenv(name);
    return value && *value;
}
}  // namespace follow
