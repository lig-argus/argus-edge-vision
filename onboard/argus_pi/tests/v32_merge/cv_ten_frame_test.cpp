#include <cassert>

#include "argus/control/target/include/cv_ten_frame_memory.hpp"
using namespace follow;
int main() {
    CvTenFrameMemory memory;
    for (unsigned frame = 1; frame <= 40; ++frame) {
        memory.remember(frame, 1000000000LL + frame * 50000000LL, 1000000000LL + frame * 50000000LL,
                        .1 * frame, -.2, true, true, false);
    }
    auto old = memory.selected(40);
    assert(old && old->frame_id == 30 && old->source_ns == 2500000000LL);
    memory.freeze_before(41);
    assert(memory.selected(40)->frame_id == 31);
    memory.remember(41, 3050000000LL, 3050000000LL, .9, 0., true, true, false);
    assert(memory.selected(41)->frame_id == 31);
    memory.clear_frozen();
    assert(memory.selected(41)->frame_id == 31);
    memory.freeze_before(100);  // Exact frame-10 evidence absent: no invented replacement.
    assert(!memory.selected(41));
    memory.clear_frozen();
    memory.remember(42, 3100000000LL, 3100000000LL, .9, 0., false, true, false);
    assert(!memory.ten_before(52));
    memory.reset();
    assert(!memory.selected(41));
}
