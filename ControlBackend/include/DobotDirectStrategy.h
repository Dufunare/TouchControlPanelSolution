#pragma once

#include "ICommunicationStrategy.h"
#include "dobot/Dashboard.h"
#include "dobot/Feedback.h"
#include "dobot/FeedbackData.h"

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace touchpanel
{
    /// Concrete ICommunicationStrategy that communicates directly with a
    /// Dobot CR-series robot arm via the official TCP protocol (V4).
    ///
    /// - Control port is provided by connect(ip, port), default 29999 when port==0.
    /// - Feedback port uses control port + 5 (e.g. 29999 -> 30004).
    ///
    /// Coordinate mapping (scale + offset) and soft-limit clamping are applied
    /// inside sendCoordinate() before the command string is assembled.
    class DobotDirectStrategy final : public ICommunicationStrategy
    {
    public:
        /// Fixed orientation values (Rx, Ry, Rz) appended to every ServoP
        /// command.  Adjust these to match the actual tool orientation.
        struct Orientation
        {
            double rx = 150.0;
            double ry = 0.0;
            double rz = 90.0;
        };

        /// Workspace soft limits (mm).  Any coordinate outside these bounds
        /// is clamped to the boundary before being sent.
        struct SoftLimits
        {
            double minX = -800.0;
            double maxX = 800.0;
            double minY = -800.0;
            double maxY = 800.0;
            double minZ = -200.0;
            double maxZ = 600.0;
        };

        /// Linear mapping applied to raw Touch coordinates before clamping:
        ///   robotCoord = rawCoord * scale + offset
        struct CoordinateMapping
        {
            double scaleX = 1.0;
            double scaleY = 1.0;
            double scaleZ = 1.0;
            double offsetX = 0.0;
            double offsetY = 0.0;
            double offsetZ = 0.0;
        };

        DobotDirectStrategy();
        ~DobotDirectStrategy() override;

        // --- ICommunicationStrategy ---
        bool connect(const std::string& ip, unsigned short port) override;
        void disconnect() override;
        bool sendCoordinate(double x, double y, double z) override;
        bool isConnected() const override;
        std::string lastError() const override;

        // --- Configuration helpers (call before connect) ---
        void setOrientation(const Orientation& o);
        void setSoftLimits(const SoftLimits& limits);
        void setCoordinateMapping(const CoordinateMapping& mapping);

        /// Returns the latest feedback data snapshot (thread-safe copy).
        Dobot::CFeedbackData latestFeedback() const;

    private:
        void feedbackLoop();

        static double clamp(double value, double lo, double hi);

        Dobot::CDashboard m_dashboard;
        Dobot::CFeedback m_feedback;

        Orientation m_orientation;
        SoftLimits m_softLimits;
        CoordinateMapping m_mapping;

        mutable std::mutex m_feedbackMutex;
        Dobot::CFeedbackData m_latestFeedback;

        std::thread m_feedbackThread;
        std::atomic<bool> m_feedbackRunning{ false };
        bool m_netInitialized = false;

        mutable std::mutex m_errorMutex;
        std::string m_lastError;
    };
} // namespace touchpanel
