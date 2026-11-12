// 테스트용 IFcEpochSource. 테스트가 fc_epoch 를 직접 증가시킨다
#ifndef ARGUS_ONBOARD_TEST_FAKE_FC_EPOCH_SOURCE_HPP_
#define ARGUS_ONBOARD_TEST_FAKE_FC_EPOCH_SOURCE_HPP_

#include <cstdint>
#include <utility>

#include "argus/core/subscription.hpp"
#include "argus/onboard/fc_epoch_source.hpp"
#include "argus/onboard/handler_list.hpp"

namespace argus::onboard {

class FakeFcEpochSource : public IFcEpochSource {
public:
    std::uint64_t fc_epoch() const override { return fc_epoch_; }

    [[nodiscard]] core::Subscription subscribe_epoch_changes(EpochHandler handler) override {
        return handlers_.add(std::move(handler));
    }

    // 테스트용: fc_epoch 를 1 증가시키고 등록된 handler 를 호출한다
    void simulate_epoch_change() {
        ++fc_epoch_;
        handlers_.call(fc_epoch_);
    }

private:
    std::uint64_t fc_epoch_ = 0;
    HandlerList<std::uint64_t> handlers_;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_TEST_FAKE_FC_EPOCH_SOURCE_HPP_
