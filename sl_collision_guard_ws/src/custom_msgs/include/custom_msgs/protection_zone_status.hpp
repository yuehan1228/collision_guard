#pragma once

#include <cstdint>

namespace custom_msgs {

enum class ProtectionZoneStatus : uint16_t {
    DISABLED = 0,
    CLEAR = 1,
    LIMIT_MEDIUM_SPEED = 2,
    LIMIT_LOW_SPEED = 3,
    STOP = 4,
};

} // namespace custom_msgs
