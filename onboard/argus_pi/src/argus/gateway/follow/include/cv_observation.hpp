// 원래 FC 정렬 실관측 기록 (Pi tools/real_fc/cv_observation.py CvObservationLedger와 같은 규칙).
// 명령을 보내거나 검출을 만들거나 관측 시각을 바꾸지 않는다. 소실 중 등속 이동(CV) 명령의 근거
// 확인용이다.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "argus/gateway/common/include/msgpack_codec.hpp"

namespace argus::gateway {

constexpr std::int64_t kCvWindowNs = 3'000'000'000;

// 원본 (source, class_id, label)
struct CvIdentity {
    std::string source;
    std::int64_t category{0};
    std::string label;
    bool operator==(const CvIdentity& other) const {
        return source == other.source && category == other.category && label == other.label;
    }
};

class CvObservationLedger {
public:
    explicit CvObservationLedger(std::size_t capacity = 2048) : capacity_(capacity) {}

    void clear();
    // tracking/tracks v5 프레임 하나. 실검출(CONFIRMED + ground)이 있으면 true
    bool observe(const Wire& frame, std::int64_t now_ns, std::int64_t fc_epoch, bool verified_fc);
    bool contains(std::int64_t fc_epoch, const Wire& generation, const Wire& track_id,
                  const Wire& stamp_ns, std::int64_t now_ns);
    std::optional<CvIdentity> identity(std::int64_t fc_epoch, const Wire& generation,
                                       const Wire& track_id, const Wire& stamp_ns,
                                       std::int64_t now_ns);
    // 비교 시험용 내부 상태
    Wire debug_state() const;

private:
    struct Key {
        std::int64_t epoch{0};
        std::int64_t generation{0};
        std::int64_t track{0};
        std::int64_t stamp{0};
        bool operator==(const Key& other) const {
            return epoch == other.epoch && generation == other.generation && track == other.track &&
                   stamp == other.stamp;
        }
    };
    struct SourceKey {
        std::int64_t epoch{0};
        std::int64_t generation{0};
        std::string source;
        bool operator==(const SourceKey& other) const {
            return epoch == other.epoch && generation == other.generation && source == other.source;
        }
    };

    void prune(std::int64_t now_ns);
    static std::optional<Key> lookup_key(std::int64_t fc_epoch, const Wire& generation,
                                         const Wire& track_id, const Wire& stamp_ns);

    std::size_t capacity_;
    std::vector<std::pair<Key, CvIdentity>> observations_;  // 원본 OrderedDict 삽입 순서
    std::vector<std::pair<SourceKey, std::pair<std::int64_t, std::int64_t>>> sources_;
    std::optional<std::int64_t> epoch_;
};

}  // namespace argus::gateway
