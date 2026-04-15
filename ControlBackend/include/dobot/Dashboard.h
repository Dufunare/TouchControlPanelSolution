#pragma once
// Simplified port of the Dashboard class from Dobot TCP-IP-CR-CPP-V4 official demo.
// Only the methods required for ServoP real-time control and error handling are included.

#include "DobotClient.h"
#include "DescartesPoint.h"
#include <mutex>

namespace Dobot
{
    class CDashboard : public CDobotClient
    {
    public:
        CDashboard();
        virtual ~CDashboard();

        std::string SendRecvMsg(std::string& str);

        std::string ClearError();
        std::string EnableRobot();
        std::string DisableRobot();
        std::string GetErrorID();
        std::string RobotMode();

        std::string ServoP(const CDescartesPoint& pt);

    protected:
        void OnConnected() override;
        void OnDisconnected() override;

    private:
        std::mutex m_mutex;
    };
} // namespace Dobot
