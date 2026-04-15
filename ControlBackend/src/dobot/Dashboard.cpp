// Simplified port of the Dashboard class from Dobot TCP-IP-CR-CPP-V4 official demo.
// Only the methods needed for ServoP real-time control and error handling are included.
#include "pch.h"
#include "dobot/Dashboard.h"

#include <sstream>

namespace Dobot
{
    CDashboard::CDashboard() = default;
    CDashboard::~CDashboard() = default;

    void CDashboard::OnConnected()
    {
    }

    void CDashboard::OnDisconnected()
    {
    }

    std::string CDashboard::SendRecvMsg(std::string& str)
    {
        std::unique_lock<std::mutex> lockValue(m_mutex);
        if (!IsConnected())
        {
            return "device does not connected!!!";
        }
        if (!SendData(str))
        {
            return "send error";
        }
        return WaitReply(10000);
    }

    std::string CDashboard::ClearError()
    {
        std::string str("ClearError()");
        return SendRecvMsg(str);
    }

    std::string CDashboard::EnableRobot()
    {
        std::string str("EnableRobot()");
        return SendRecvMsg(str);
    }

    std::string CDashboard::DisableRobot()
    {
        std::string str("DisableRobot()");
        return SendRecvMsg(str);
    }

    std::string CDashboard::GetErrorID()
    {
        std::string str("GetErrorID()");
        return SendRecvMsg(str);
    }

    std::string CDashboard::RobotMode()
    {
        std::string str("RobotMode()");
        return SendRecvMsg(str);
    }

    std::string CDashboard::ServoP(const CDescartesPoint& pt)
    {
        std::ostringstream oss;
        oss << "ServoP(" << pt.x << ',' << pt.y << ',' << pt.z << ','
            << pt.rx << ',' << pt.ry << ',' << pt.rz << ')';
        std::string str = oss.str();
        return SendRecvMsg(str);
    }
} // namespace Dobot
