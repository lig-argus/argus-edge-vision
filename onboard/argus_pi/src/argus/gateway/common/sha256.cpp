#include "argus/gateway/common/include/sha256.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace argus::gateway {

namespace {

constexpr std::array<std::uint32_t, 64> kRound{
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

constexpr std::uint32_t rotate_right(std::uint32_t value, int count) {
    return (value >> count) | (value << (32 - count));
}

void compress(std::array<std::uint32_t, 8>& state, const std::vector<std::uint8_t>& message,
              std::size_t offset) {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t i = 0; i < 16; ++i) {
        words.at(i) = (static_cast<std::uint32_t>(message.at(offset + (i * 4))) << 24) |
                      (static_cast<std::uint32_t>(message.at(offset + (i * 4) + 1)) << 16) |
                      (static_cast<std::uint32_t>(message.at(offset + (i * 4) + 2)) << 8) |
                      static_cast<std::uint32_t>(message.at(offset + (i * 4) + 3));
    }
    for (std::size_t i = 16; i < 64; ++i) {
        const std::uint32_t s0 = rotate_right(words.at(i - 15), 7) ^
                                 rotate_right(words.at(i - 15), 18) ^ (words.at(i - 15) >> 3);
        const std::uint32_t s1 = rotate_right(words.at(i - 2), 17) ^
                                 rotate_right(words.at(i - 2), 19) ^ (words.at(i - 2) >> 10);
        words.at(i) = words.at(i - 16) + s0 + words.at(i - 7) + s1;
    }
    std::array<std::uint32_t, 8> v = state;
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint32_t s1 =
            rotate_right(v.at(4), 6) ^ rotate_right(v.at(4), 11) ^ rotate_right(v.at(4), 25);
        const std::uint32_t choose = (v.at(4) & v.at(5)) ^ (~v.at(4) & v.at(6));
        const std::uint32_t t1 = v.at(7) + s1 + choose + kRound.at(i) + words.at(i);
        const std::uint32_t s0 =
            rotate_right(v.at(0), 2) ^ rotate_right(v.at(0), 13) ^ rotate_right(v.at(0), 22);
        const std::uint32_t majority =
            (v.at(0) & v.at(1)) ^ (v.at(0) & v.at(2)) ^ (v.at(1) & v.at(2));
        const std::uint32_t t2 = s0 + majority;
        v.at(7) = v.at(6);
        v.at(6) = v.at(5);
        v.at(5) = v.at(4);
        v.at(4) = v.at(3) + t1;
        v.at(3) = v.at(2);
        v.at(2) = v.at(1);
        v.at(1) = v.at(0);
        v.at(0) = t1 + t2;
    }
    for (std::size_t i = 0; i < 8; ++i) {
        state.at(i) += v.at(i);
    }
}

}  // namespace

std::string sha256_hex(const std::string& data) {
    std::array<std::uint32_t, 8> state{0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                       0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::vector<std::uint8_t> message(data.begin(), data.end());
    const std::uint64_t bit_length = static_cast<std::uint64_t>(data.size()) * 8;
    message.push_back(0x80);
    while (message.size() % 64 != 56) {
        message.push_back(0);
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
        message.push_back(static_cast<std::uint8_t>(bit_length >> shift));
    }
    for (std::size_t offset = 0; offset < message.size(); offset += 64) {
        compress(state, message, offset);
    }
    static constexpr std::array<char, 16> kHexDigits{'0', '1', '2', '3', '4', '5', '6', '7',
                                                     '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    std::string out;
    for (const std::uint32_t word : state) {
        for (int shift = 28; shift >= 0; shift -= 4) {
            out += kHexDigits.at((word >> static_cast<unsigned>(shift)) & 0x0fU);
        }
    }
    return out;
}

}  // namespace argus::gateway
