#pragma once

#include <array>
#include <cstdint>

namespace touchpanel
{
    struct DeviceState
    {
        std::array<double, 3> positionMm{ 0.0, 0.0, 0.0 };

        // HD_CURRENT_TRANSFORM is column-major; 3x3 is converted to row-major here.
        std::array<double, 9> rotationMatrix{ 1,0,0,0,1,0,0,0,1 };
        bool orientationValid = false;
        bool deviceCalibrated = false;
        bool calibrationNeedsManualInput = false;
        std::uint64_t calibrationEpoch = 0;
        bool initialized = false;
        bool valid = false;
        bool schedulerRunning = false;
        bool button1Pressed = false;

        std::uint64_t sampleCounter = 0;
    };
} // namespace touchpanel
