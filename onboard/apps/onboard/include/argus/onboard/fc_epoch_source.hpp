// 현재 fc_epoch 와 그 변경을 제공하는 인터페이스
// 규약: docs/spec/process_interfaces.md 3장 (fc_epoch)
// 구현: FcLinkTracker, 테스트용 FakeFcEpochSource (onboard test/)
// 사용: fc_epoch 에 묶인 값(FC 시각의 변환값, FC 제어 명령 송신권, 진행 중인 요청)을 가진 클래스
#ifndef ARGUS_ONBOARD_FC_EPOCH_SOURCE_HPP_
#define ARGUS_ONBOARD_FC_EPOCH_SOURCE_HPP_

#include <cstdint>
#include <functional>

#include "argus/core/subscription.hpp"

namespace argus::onboard {

class IFcEpochSource {
public:
    using EpochHandler = std::function<void(std::uint64_t)>;

    IFcEpochSource() = default;
    virtual ~IFcEpochSource() = default;
    IFcEpochSource(const IFcEpochSource&) = delete;
    IFcEpochSource& operator=(const IFcEpochSource&) = delete;
    IFcEpochSource(IFcEpochSource&&) = delete;
    IFcEpochSource& operator=(IFcEpochSource&&) = delete;

    // FC 연결의 현재 epoch. FC 메시지를 한 번도 받지 못한 경우 0
    virtual std::uint64_t fc_epoch() const = 0;

    // fc_epoch 가 증가할 때마다 새 값으로 handler 를 호출한다. 이전 fc_epoch 에서 얻은 값을 가진
    // 클래스는 이 호출에서 그 값을 폐기한다.
    // 반환된 Subscription 이 소멸하면 등록이 해제된다
    [[nodiscard]] virtual core::Subscription subscribe_epoch_changes(EpochHandler handler) = 0;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_FC_EPOCH_SOURCE_HPP_
