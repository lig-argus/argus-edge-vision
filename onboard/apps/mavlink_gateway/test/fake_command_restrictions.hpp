// 테스트용 ILandingLockState, IRcOverrideState. 테스트가 값을 직접 정한다
#ifndef ARGUS_ONBOARD_TEST_FAKE_COMMAND_RESTRICTIONS_HPP_
#define ARGUS_ONBOARD_TEST_FAKE_COMMAND_RESTRICTIONS_HPP_

#include "argus/mavlink_gateway/landing_lock_state.hpp"
#include "argus/mavlink_gateway/rc_override_state.hpp"

namespace argus::mavlink_gateway {

class FakeLandingLock : public ILandingLockState {
public:
    bool active() const override { return locked; }

    bool locked = false;
};

class FakeRcOverride : public IRcOverrideState {
public:
    bool pilot_in_control() const override { return pilot; }

    bool pilot = false;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_TEST_FAKE_COMMAND_RESTRICTIONS_HPP_
