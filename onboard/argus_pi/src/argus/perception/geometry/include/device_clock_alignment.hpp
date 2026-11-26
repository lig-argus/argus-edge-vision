#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <utility>

namespace follow {

// Receipt lower-envelope estimate. It removes variable delivery delay, not the
// unknown fixed sensor/USB delay; this must never certify exposure alignment.
class DeviceClockAlignment {
public:
    bool observe(std::uint32_t milliseconds, std::int64_t receipt_ns) {
        if (receipt_ns <= 0 || receipt_ns <= last_receipt_) {
            return ready_;
        }
        const auto delta = initialized_ ? difference(milliseconds, raw_) : 0;
        if (initialized_ && delta < -100) {
            // A late packet is not a reboot. The caller resets explicitly on a
            // verified FC epoch/source change; never re-anchor old data to now.
            return ready_;
        }
        if (initialized_ && delta > 0 &&
            (delta > std::max<std::int64_t>(1000, (receipt_ns - last_receipt_) / 500000) ||
             tick_ > ((std::numeric_limits<std::int64_t>::max() / 1000000) - delta))) {
            return ready_;
        }
        if (!initialized_) {
            initialized_ = true;
            raw_ = milliseconds;
            tick_ = milliseconds;
            first_tick_ = tick_;
            first_receipt_ = receipt_ns;
        } else if (delta >= 0) {
            tick_ += delta;
            raw_ = milliseconds;
        } else {
            // Small out-of-order MAVLink packets cannot move the clock origin.
            return ready_;
        }
        const auto offset = receipt_ns - (tick_ * 1000000);
        offsets_.emplace_back(receipt_ns, offset);
        while (offsets_.size() > 1 &&
               (receipt_ns - offsets_.front().first > 2000000000 || offsets_.size() > 512)) {
            offsets_.pop_front();
        }
        minimum_offset_ = std::min_element(offsets_.begin(), offsets_.end(),
                                           [](const auto& left, const auto& right) {
                                               return left.second < right.second;
                                           })
                              ->second;
        ++samples_;
        const auto elapsed = receipt_ns - first_receipt_;
        const auto device_elapsed = (tick_ - first_tick_) * 1000000;
        if (!ready_ && samples_ >= 10 && elapsed >= 200000000 && device_elapsed >= 150000000 &&
            static_cast<double>(device_elapsed) >= static_cast<double>(elapsed) * .8 &&
            static_cast<double>(device_elapsed) <= static_cast<double>(elapsed) * 1.2) {
            offset_ = minimum_offset_;
            ready_ = true;
        } else if (ready_) {
            if (minimum_offset_ < offset_) {
                // 2026-11-12: 받은 표본이 추정보다 이르면(수신이 생성보다 앞설 수 없으므로)
                // 추정이 늦은 것이 확실하다. 바로 내린다. 시작 200ms 창에서 늦게 잡힌 추정이
                // 100ppm으로만 고쳐지며 수십 ms 어긋남이 이어지던 문제(2026-11-12 16:13 실내
                // 추적 프레임 거부)를 막는다.
                offset_ = minimum_offset_;
            } else {
                // Slew correction <=100 ppm; delivery bursts cannot jump the clock later.
                const auto limit = std::max<std::int64_t>(1, (receipt_ns - last_receipt_) / 10000);
                offset_ += std::min(minimum_offset_ - offset_, limit);
            }
        }
        last_receipt_ = receipt_ns;
        return ready_;
    }

    std::optional<std::int64_t> map(std::uint32_t milliseconds) const {
        if (!ready_) {
            return std::nullopt;
        }
        const auto tick = tick_ + difference(milliseconds, raw_);
        if (tick < 0 || tick > std::numeric_limits<std::int64_t>::max() / 1000000) {
            return std::nullopt;
        }
        const auto ns = tick * 1000000;
        if ((offset_ > 0 && ns > std::numeric_limits<std::int64_t>::max() - offset_) ||
            (offset_ < 0 && ns <= -offset_)) {
            return std::nullopt;
        }
        return ns + offset_;
    }

    // 진단용 (정렬 판단에는 쓰지 않는다).
    bool ready() const { return ready_; }
    std::int64_t offset_ns() const { return offset_; }
    std::int64_t minimum_offset_ns() const { return minimum_offset_; }
    unsigned sample_count() const { return samples_; }

private:
    static std::int64_t difference(std::uint32_t a, std::uint32_t b) {
        auto delta = static_cast<std::int64_t>(a) - static_cast<std::int64_t>(b);
        if (delta > 2147483647LL) {
            delta -= 4294967296LL;
        } else if (delta < -2147483648LL) {
            delta += 4294967296LL;
        }
        return delta;
    }
    bool initialized_{false}, ready_{false};
    std::uint32_t raw_{};
    std::int64_t tick_{}, first_tick_{}, offset_{}, minimum_offset_{};
    std::int64_t last_receipt_{}, first_receipt_{};
    std::deque<std::pair<std::int64_t, std::int64_t>> offsets_;
    unsigned samples_{};
};

}  // namespace follow
