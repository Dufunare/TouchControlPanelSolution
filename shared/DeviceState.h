#pragma once

#include <array>
#include <cstdint>

namespace touchpanel
{
    struct DeviceState
    {
        std::array<double, 3> positionMm{ 0.0, 0.0, 0.0 };

        bool initialized = false;
        bool valid = false;
        bool schedulerRunning = false;

        std::uint64_t sampleCounter = 0;
    };
} // namespace touchpanel
