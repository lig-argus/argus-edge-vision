// ZmqBus 가 값을 전송 바이트로 바꾸는 데 쓰는 변환 함수의 표.
// ZmqBus 는 메시지 타입을 알지 못한다. 실행 파일이 쓰는 타입마다 변환 함수를 이 표에 넣어 ZmqBus
// 의 생성자에 넘긴다 (예: vehicle_api_bus 의 zmq_codecs()). 표에 없는 타입으로는 발행자, 구독자,
// 요청하는 쪽, 응답하는 쪽을 만들 수 없다.
// 규약: docs/spec/process_interfaces.md 3장 (직렬화한 본문 하나와, 필요하면 본문과 따로 보내는
// 바이트 열 하나)
#ifndef ARGUS_BUS_ZMQ_ZMQ_CODEC_HPP_
#define ARGUS_BUS_ZMQ_ZMQ_CODEC_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <typeindex>
#include <utility>
#include <vector>

namespace argus::bus_zmq {

using Bytes = std::vector<std::uint8_t>;

// 타입 하나의 변환 함수 둘. value 는 표의 키와 같은 타입의 값을 가리킨다
struct ZmqCodec {
    // 값을 본문(body)과 바이너리(binary)로 바꾼다. body 와 binary 는 비어 있는 상태로 받는다.
    // binary 를 비워 두면 바이너리 프레임을 보내지 않는다. sequence 는 발행자가 발행마다 1 씩
    // 올린 번호이며 본문에 싣는다 (요청과 응답에서는 0 이고 싣지 않는다). 바꿀 수 없으면 false
    std::function<bool(const void* value, std::uint64_t sequence, Bytes& body, Bytes& binary)>
        encode;

    // 받은 본문과 바이너리를 value 에 채우고, 본문에 실린 번호를 sequence 에 넣는다. value 는
    // 기본값으로 만든 값을 가리킨다. 바이너리 프레임이 없었으면 binary 는 비어 있다. 형식이 맞지
    // 않으면 false 이고, ZmqBus 는 그 메시지를 폐기한다
    std::function<bool(const Bytes& body, const Bytes& binary, void* value,
                       std::uint64_t& sequence)>
        decode;
};

// C++ 값 타입 → 변환 함수
using ZmqCodecTable = std::map<std::type_index, ZmqCodec>;

// 타입 T 의 변환 함수를 표에 넣는다. 같은 타입이 이미 있으면 바꾼다.
// encode 는 bool(const T& value, std::uint64_t sequence, Bytes& body, Bytes& binary),
// decode 는 bool(const Bytes& body, const Bytes& binary, T& value, std::uint64_t& sequence)
// 로 부를 수 있어야 한다. void* 를 T 로 바꾸는 코드를 이 함수 한 곳에만 둔다
template <typename T, typename Encode, typename Decode>
void add_zmq_codec(ZmqCodecTable& table, Encode encode, Decode decode) {
    ZmqCodec codec;
    codec.encode = [encode = std::move(encode)](const void* value, std::uint64_t sequence,
                                                Bytes& body, Bytes& binary) -> bool {
        return encode(*static_cast<const T*>(value), sequence, body, binary);
    };
    codec.decode = [decode = std::move(decode)](const Bytes& body, const Bytes& binary, void* value,
                                                std::uint64_t& sequence) -> bool {
        return decode(body, binary, *static_cast<T*>(value), sequence);
    };
    table.insert_or_assign(std::type_index(typeid(T)), std::move(codec));
}

}  // namespace argus::bus_zmq

#endif  // ARGUS_BUS_ZMQ_ZMQ_CODEC_HPP_
