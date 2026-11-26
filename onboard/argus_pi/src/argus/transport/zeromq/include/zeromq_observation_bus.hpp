// observations 토픽용 ZeroMQ 역할 포트. 소켓은 생성한 작업자 스레드에서만 사용한다.
#pragma once

#include <chrono>
#include <string>

#include "argus/perception/observations/include/observation_v2.hpp"

namespace follow {

class ZeroMqContext final {
public:
    ZeroMqContext();
    ~ZeroMqContext();
    ZeroMqContext(const ZeroMqContext&) = delete;
    ZeroMqContext& operator=(const ZeroMqContext&) = delete;
    void* native_handle() const noexcept { return context_; }

private:
    void* context_{nullptr};
};

enum class BusSocketEndpoint { Connect, Bind };

class ZeroMqObservationPublisher final {
public:
    ZeroMqObservationPublisher(ZeroMqContext& context, const std::string& endpoint,
                               BusSocketEndpoint action = BusSocketEndpoint::Connect,
                               int send_hwm = 10);
    ~ZeroMqObservationPublisher();
    ZeroMqObservationPublisher(const ZeroMqObservationPublisher&) = delete;
    ZeroMqObservationPublisher& operator=(const ZeroMqObservationPublisher&) = delete;

    // PUB는 구독자 도착을 확인해 주지 않는다. 호출 성공은 소켓이 메시지를 받았다는 뜻뿐이다.
    bool publish(const ObservationV2& message, std::string& error);

private:
    void* socket_{nullptr};
};

struct ObservationBusReceive {
    enum class State { Message, Timeout, Invalid, TransportError } state{State::Timeout};
    ObservationV2 message;
    std::string error;
};

class ZeroMqObservationSubscriber final {
public:
    ZeroMqObservationSubscriber(ZeroMqContext& context, const std::string& endpoint,
                                BusSocketEndpoint action = BusSocketEndpoint::Connect,
                                int receive_hwm = 2);
    ~ZeroMqObservationSubscriber();
    ZeroMqObservationSubscriber(const ZeroMqObservationSubscriber&) = delete;
    ZeroMqObservationSubscriber& operator=(const ZeroMqObservationSubscriber&) = delete;

    // 호출자는 짧은 timeout으로 반복해 입력 정지 중에도 감시/제어 주기를 계속해야 한다.
    ObservationBusReceive receive(std::chrono::milliseconds timeout);

private:
    void* socket_{nullptr};
};

inline constexpr char kObservationsTopic[] = "perception/observations";

}  // namespace follow
