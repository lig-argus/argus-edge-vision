// Additive reporting only: never use this projection to authorize flight actions.
#pragma once
#include "argus/common/serialization/include/native_json_io.hpp"
namespace follow {
inline void add_command_outcome(NativeJson& result) {
    const bool pending = result.value("pending", false);
    const bool success = result.value("accepted", false);
    const bool admitted = result.value(
        "request_accepted", pending || success || result.value("execution_complete", false));
    result["request_accepted"] = admitted;
    result["outcome"] = pending    ? "IN_PROGRESS"
                        : success  ? "SUCCEEDED"
                        : admitted ? "FAILED"
                                   : "REJECTED";
}
}  // namespace follow
