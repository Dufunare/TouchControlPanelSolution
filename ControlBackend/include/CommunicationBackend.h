#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace touchpanel
{
    class CommunicationBackend
    {
    public:
        using MessageCallback = std::function<void(const std::string&)>;
        using ConnectionCallback = std::function<void(bool connected)>;
        using TextCallback = std::function<void(const std::string&)>;
        using OperationCallback = std::function<void(const std::string& operation, bool success, const std::string& detail)>;

        CommunicationBackend();
        ~CommunicationBackend();

        CommunicationBackend(const CommunicationBackend&) = delete;
        CommunicationBackend& operator=(const CommunicationBackend&) = delete;

        void setMessageCallback(MessageCallback callback);
        void setConnectionCallback(ConnectionCallback callback);
        void setRobotStatusCallback(TextCallback callback);
        void setTcpTxCallback(TextCallback callback);
        void setTcpRxCallback(TextCallback callback);
        void setOperationCallback(OperationCallback callback);

        void connectTransit(const std::string& ipOverride = {}, std::uint16_t portOverride = 0);
        void disconnectTransit();

        void powerOn();
        void enableRobot();
        void disableRobot();
        void clearError();
        void emergencyStop();
        void startDrag();
        void stopDrag();

    private:
        struct Impl;
        Impl* m_impl = nullptr;
    };
}
