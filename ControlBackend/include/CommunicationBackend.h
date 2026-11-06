#pragma once

#include <cstdint>
#include <array>
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
        using JointSolutionCallback = std::function<void(bool success, const std::array<double,6>& jointAngles, const std::string& detail)>;

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
        void setJointSolutionCallback(JointSolutionCallback callback);

        void connectTransit(const std::string& ipOverride = {}, std::uint16_t portOverride = 0);
        void disconnectTransit();
        bool isTransitConnected() const;

        void sendMotion(double x, double y, double z); // Deprecated ServoP: always disabled.
        void requestInverseSolution(const std::array<double,6>& pose,int user,int tool,
                                    const std::array<double,6>& seed);
        void sendLinearStep(const std::array<double,6>& pose,int user,int tool);
        bool sendServoJointTarget(const std::array<double,6>& joints);
        // Distinguish an unsafe XYZ request from a broken connection.
        // The control layer can hold at an experimental software boundary
        // without ending the clutch, but a transport error must abort.
        enum class ServoPSendCode {
            Sent, OutsideWorkspace, InvalidTarget, Disconnected, TransportFailed
        };
        struct ServoPSendResult {
            ServoPSendCode code=ServoPSendCode::InvalidTarget;
            int axis=-1;
            double value=0,minimum=0,maximum=0;
        };
        // Legacy 8888 -> 30003, non-blocking online Cartesian ServoP.
        // Actual robot RPY is held by the caller. Never clamp XYZ silently.
        ServoPSendResult sendServoCartesianTarget(const std::array<double,6>& pose);
        void requestControlledStop();
        void requestRobotMode();
        void requestCurrentPose();
        bool isRobotModeReady() const;
        bool tryGetLastPose(std::array<double, 6>& pose) const;

        void powerOn();
        void enableRobot();
        void disableRobot();
        void clearError();
        void emergencyStop();
        void startDrag();
        void stopDrag();

    private:
        void disconnectTransitInternal(bool silentIfNotConnected);

        struct Impl;
        Impl* m_impl = nullptr;
    };
}
