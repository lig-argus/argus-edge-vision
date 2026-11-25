// 다른 스레드(Fast DDS 의 수신 스레드)가 process_events() 를 부르는 스레드를 깨우는 pipe.
// 라이브러리 밖에서는 include 하지 않는다
#ifndef ARGUS_BUS_DDS_SRC_WAKEUP_PIPE_HPP_
#define ARGUS_BUS_DDS_SRC_WAKEUP_PIPE_HPP_

#include <cstdint>

namespace argus::bus_dds::detail {

// notify() 는 어느 스레드에서 불러도 된다. 나머지는 process_events() 를 부르는 스레드에서 부른다.
// 두 끝 모두 non-blocking 이다
class WakeupPipe {
public:
    WakeupPipe();
    ~WakeupPipe();
    WakeupPipe(const WakeupPipe&) = delete;
    WakeupPipe& operator=(const WakeupPipe&) = delete;
    WakeupPipe(WakeupPipe&&) = delete;
    WakeupPipe& operator=(WakeupPipe&&) = delete;

    // pipe 를 만들지 못했으면 false
    bool valid() const { return read_fd_ >= 0; }

    // IFdWatcher 에 등록할 읽는 쪽 fd
    std::int32_t read_fd() const { return read_fd_; }

    // 1 바이트를 기록한다. pipe 가 가득 차 있으면 이미 깨울 바이트가 있으므로 기록하지 않는다
    void notify() const;

    // 쌓인 바이트를 모두 읽어 버린다
    void drain() const;

private:
    std::int32_t read_fd_ = -1;
    std::int32_t write_fd_ = -1;
};

}  // namespace argus::bus_dds::detail

#endif  // ARGUS_BUS_DDS_SRC_WAKEUP_PIPE_HPP_
