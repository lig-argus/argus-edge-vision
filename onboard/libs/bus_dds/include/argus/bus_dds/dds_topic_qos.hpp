// DdsBus 가 토픽마다 발행자(DataWriter)에 설정하는 QoS 값의 표.
// DdsBus 는 토픽의 주기를 알지 못한다. 토픽을 정의한 모듈이 자기 토픽의 값을 이 표로 내놓고, 실행
// 파일의 main() 이 표들을 합쳐 DdsBus::create() 에 전달한다 (DdsTypeTable 과 같은 방식).
// 규약: docs/spec/process_interfaces.md 3장 (토픽별 QoS)
//
// 규칙
// - 값은 발행자에만 설정하고 구독자(DataReader)는 기본값(무한)으로 둔다. DDS 의 요청-제공 호환
//   규칙상 구독자가 요구한 기한보다 긴 값을 제공하는 발행자와는 연결되지 않으므로, 구독자에도 두면
//   DdsBus 를 사용하지 않는 프로세스의 발행과 통신이 끊긴다.
// - 표에 없는 토픽과 요청-응답의 토픽은 Deadline 을 두지 않는다 (무한).
// - Liveliness 는 두지 않는다. Fast DDS 는 구독자 자신의 lease 가 유한할 때만 발행자의 소실을
//   판정하므로 구독자에도 설정해야 하는데, 그러면 위의 연결 단절이 생긴다.
#ifndef ARGUS_BUS_DDS_DDS_TOPIC_QOS_HPP_
#define ARGUS_BUS_DDS_DDS_TOPIC_QOS_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <optional>
#include <string>

namespace argus::bus_dds {

struct DdsTopicQos {
    // 발행 간격의 상한 (DDS DEADLINE). 넘으면 발행하는 쪽의 DdsBus 가 로그에 남긴다.
    // 주기가 고정된 토픽에 둔다. 없으면 무한
    std::optional<std::chrono::nanoseconds> deadline;
};

// 토픽 이름 → 값
using DdsTopicQosTable = std::map<std::string, DdsTopicQos, std::less<>>;

}  // namespace argus::bus_dds

#endif  // ARGUS_BUS_DDS_DDS_TOPIC_QOS_HPP_
