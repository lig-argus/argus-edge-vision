#pragma once
// 게이트웨이의 FC 텔레메트리 6370 (2026-11-25 batch 6a, 팀 6370 전환).
// 추종 송신부(FollowBOwner)는 지금처럼 fc/state, fc/attitude, fc/mode_event 의 msgpack 을 만든다.
// 실제 전송(ZmqFollowTransport)만 이 코드로 팀 규약 [이름][protobuf] 을 보낸다. 그래서 비교 시험
// (IFollowTransport 를 바꿔 끼운 stage 시험, differential)의 msgpack 내용은 그대로다.
// - FcTelemetryWire: [이름][msgpack] -> 팀 seq (토픽별 발행 번호) -> NativeJson -> to_proto ->
//   보내기. 팀 ZmqBus 는 쓰지 않는다: 발행이 FC 링크 잠금을 잡은 여러 스레드 (FlightLink 소유
//   스레드, MainGcsBridge 의 worker 와 io 스레드의 invalidate) 에서 일어나 팀 ZmqBus 의
//   "process_events 스레드 하나" 규칙을 지킬 수 없다.
//   2026-11-25 6a 리뷰: 변환 (msgpack 해석, to_proto, 직렬화) 이 FC 링크 잠금 안에서 fc/state 한
//   번에 평균 약 0.7 ms, 부하에서 최대 6.6 ms 를 잡았다. 그래서 publish 는 잠금 안에서 이름 검사,
//   팀 seq 정하기, 대기열 넣기만 하고, 변환과 send 는 송신 스레드 하나가 대기열 순서대로 한다 (토픽
//   사이 순서와 토픽별 seq 순서가 지켜진다). PUB 소켓은 start 뒤로 송신 스레드만 쓴다.
//   - 팀 seq 는 발행 시도마다 쓴다 (팀 ZmqBus 의 ++sequence_ 와 같다): msgpack 해석 실패, to_proto
//     실패, 상한 초과, 대기열 가득 참도 번호를 쓴다.
//   - 변환 실패는 보내지 않고 세어 1, 2, 4 ... 번째만 std::cerr 에 남긴다 (받는 쪽은 신선도가 끊겨
//     무효가 된다). 대기열 가득 참은 PUB HWM 유실처럼 새 메시지를 버리고 QueueFull 을 돌려준다.
//   - zmq_send 실패는 송신 스레드가 남기고 다음 publish 가 SendFailed 로 한 번 알린다 (예전 send 의
//     예외가 한 발행 늦게 난다).
// - fc_state_msgpack: 정비 지상 확인용. [fc/state][protobuf] 본문을 Pi 게이트웨이가 예전에 보내던
//   msgpack 바이트로 되살린다. 받는 쪽은 예전처럼 msgpack_unpack 뒤 같은 키 검사를 한다.
// 이 헤더는 nlohmann 을 포함하지 않는다 (tracks_protobuf.hpp, vision_protobuf.hpp 와 같은 까닭으로
// 변환은 argus_gateway_fc_pb 라이브러리에 둔다).
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace argus::gateway {

enum class FcPublish : std::uint8_t {
    Queued,      // 송신 대기열에 넣었다 (팀 seq 를 정했다. 변환 실패는 송신 스레드가 센다)
    NotFcTopic,  // 이름이 fc/state, fc/attitude, fc/mode_event 가 아님 (보내지 않음)
    QueueFull,   // 대기열이 가득 차 이 메시지를 버렸다 (팀 seq 는 쓴다)
    SendFailed,  // 앞선 발행의 zmq_send 실패, 또는 송신 스레드가 없음
};

struct FcWireStats {
    std::uint64_t queued = 0;
    std::uint64_t sent = 0;
    std::uint64_t encode_failures = 0;  // msgpack 해석, to_proto, 상한 초과
    std::uint64_t queue_drops = 0;
    std::uint64_t send_failures = 0;
};

class FcTelemetryWire {
public:
    static constexpr std::size_t kQueueDepth = 64;  // 6370 SNDHWM 과 같은 값

    FcTelemetryWire();
    ~FcTelemetryWire();
    FcTelemetryWire(const FcTelemetryWire&) = delete;
    FcTelemetryWire& operator=(const FcTelemetryWire&) = delete;
    FcTelemetryWire(FcTelemetryWire&&) = delete;
    FcTelemetryWire& operator=(FcTelemetryWire&&) = delete;

    // socket: libzmq PUB 소켓 (void*). 송신 스레드를 시작하고 소켓을 넘긴다 (한 번만)
    void start(void* socket, std::size_t queue_depth = kQueueDepth);
    // topic, packed: FollowBOwner 가 만든 두 프레임. FC 링크 잠금 안에서 부른다
    FcPublish publish(const std::vector<std::uint8_t>& topic,
                      const std::vector<std::uint8_t>& packed, std::string& error);
    // 대기열을 모두 보낸 뒤 송신 스레드를 끝낸다. 소켓을 닫기 전에 부른다 (여러 번 불러도 된다)
    void stop();
    FcWireStats stats() const;
    // 마지막으로 쓴 팀 seq (아직 발행하지 않았으면 0)
    std::uint64_t last_sequence(const std::string& topic) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

enum class FcStateFrame : std::uint8_t {
    Ok,          // packed 에 msgpack 바이트
    NotFcState,  // [이름][본문] 이 아니거나 이름이 정확히 fc/state 가 아님 (건너뛴다)
    Invalid,     // 크기 상한, 빈 본문, protobuf 해석, to_json 실패 (status 없는 팀 본문 포함)
};

FcStateFrame fc_state_msgpack(const std::vector<std::vector<std::uint8_t>>& parts,
                              std::vector<std::uint8_t>& packed, std::string& error);

}  // namespace argus::gateway
