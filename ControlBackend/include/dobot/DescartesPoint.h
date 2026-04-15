#pragma once
// Ported from Dobot TCP-IP-CR-CPP-V4 official demo.

#include <sstream>
#include <string>

namespace Dobot
{
    struct CDescartesPoint
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double rx = 0.0;
        double ry = 0.0;
        double rz = 0.0;

        std::string ToString()
        {
            std::ostringstream oss;
            oss << x << ',' << y << ',' << z << ',' << rx << ',' << ry << ',' << rz;
            return oss.str();
        }
    };
} // namespace Dobot
