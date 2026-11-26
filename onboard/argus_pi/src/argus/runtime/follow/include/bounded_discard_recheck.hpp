#pragma once

#include <utility>

namespace follow {
// Recompute from fresh state once; never resend the discarded candidate.
template <typename Status, typename Recompute>
Status bounded_discard_recheck(Status initial, bool enabled, Recompute&& recompute) {
    if (enabled && initial.mission == "COMMAND_DISCARDED" &&
        initial.reason == "selection_epoch_changed_before_vehicle_send") {
        return std::forward<Recompute>(recompute)();
    }
    return initial;
}
}  // namespace follow
