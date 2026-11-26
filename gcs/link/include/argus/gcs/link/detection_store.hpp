// 센서별 최근 탐지 결과. 영상 프레임과 짝짓는 최근 기록, 마지막 결과, 초당 수신 수(fps)를 둔다.
// 입력은 영상 수신 쪽 어댑터가 메타데이터에서 만든 DetectionFrame 이다 (detection.hpp).
// 순서 규칙: 같은 센서에서 source 와 generation 이 마지막 결과와 같으면 frameId 가 커지는 결과만
// 받는다. 같거나 작은 frameId 는 중복이나 늦게 도착한 결과이므로 버린다. generation 이 작아진
// 결과도 같은 source 의 옛 결과이므로 버린다. source 가 바뀌거나 generation 이 커지면 새 문맥으로
// 보고 그 센서의 기록과 fps 를 비운다 (옛 문맥의 박스를 새 영상에 짝짓지 않게). 송신 쪽이 다시
// 시작해 generation 이 처음부터 다시 붙는 경우는 영상 수신 쪽이 RTSP 세션이 바뀐 것을 알고 clear 를
// 부른다
#ifndef ARGUS_GCS_LINK_DETECTION_STORE_HPP_
#define ARGUS_GCS_LINK_DETECTION_STORE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>

#include "argus/gcs/link/detection.hpp"
#include "argus/gcs/link/link_protocol.hpp"

namespace argus::gcs::link {

class DetectionStore {
public:
    // 센서별로 남기는 최근 결과 수. 20 fps 이면 약 1.5 s 로, 영상과 메타데이터의 도착 차이를 덮는다
    static constexpr std::size_t kHistory = 30;

    // fps 를 세는 구간
    static constexpr std::int64_t kFpsWindowNs = kNsPerSecond;

    // 탐지 결과 한 프레임. receivedNs 와 fps 를 채워 기록하고 그 결과를 돌려준다. 센서가 IR, EO 가
    // 아니거나 순서 규칙에 맞지 않으면 버리고 nullopt
    std::optional<DetectionFrame> add(std::uint8_t sensor, DetectionFrame frame,
                                      std::int64_t nowNs);

    // 그 센서의 기록과 fps 를 비운다 (RTSP 세션이 바뀜)
    void clear(std::uint8_t sensor);

    // 그 센서의 마지막 결과
    std::optional<DetectionFrame> latest(std::uint8_t sensor) const;

    // 그 센서의 frameId 결과 (최근 kHistory 개 안에서)
    std::optional<DetectionFrame> forFrame(std::uint8_t sensor, std::uint32_t frameId) const;

    // 그 센서의 최근 결과 (오래된 것부터)
    const std::deque<DetectionFrame>& history(std::uint8_t sensor) const;

private:
    // 순서 규칙에 맞는가. 새 문맥이면 그 센서의 기록을 비운다
    bool accept(std::uint8_t sensor, const DetectionFrame& frame);
    void record(std::uint8_t sensor, DetectionFrame& frame);

    std::map<std::uint8_t, std::deque<std::int64_t>> times_;  // fps 구간 안의 수신 시각
    std::map<std::uint8_t, std::deque<DetectionFrame>> history_;
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_DETECTION_STORE_HPP_
