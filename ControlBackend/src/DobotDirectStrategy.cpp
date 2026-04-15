#include "pch.h"
#include "DobotDirectStrategy.h"

#include <algorithm>
#include <iostream>
#include <sstream>

namespace touchpanel
{
    DobotDirectStrategy::DobotDirectStrategy() = default;

    DobotDirectStrategy::~DobotDirectStrategy()
    {
        disconnect();
    }

    bool DobotDirectStrategy::connect(const std::string& ip, unsigned short /*port*/)
    {
        // Initialise Winsock once.
        Dobot::CDobotClient::InitNet();

        constexpr unsigned short kControlPort  = 29999;
        constexpr unsigned short kFeedbackPort = 30004;

        if (!m_dashboard.Connect(ip, kControlPort))
        {
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError = "Failed to connect to Dashboard (port 29999) at " + ip;
            return false;
        }

        if (!m_feedback.Connect(ip, kFeedbackPort))
        {
            m_dashboard.Disconnect();
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError = "Failed to connect to Feedback (port 30004) at " + ip;
            return false;
        }

        // Start the local feedback polling thread.
        m_feedbackRunning.store(true, std::memory_order_relaxed);
        m_feedbackThread = std::thread(&DobotDirectStrategy::feedbackLoop, this);

        {
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError.clear();
        }

        return true;
    }

    void DobotDirectStrategy::disconnect()
    {
        m_feedbackRunning.store(false, std::memory_order_relaxed);
        if (m_feedbackThread.joinable())
        {
            m_feedbackThread.join();
        }

        m_dashboard.Disconnect();
        m_feedback.Disconnect();
    }

    bool DobotDirectStrategy::sendCoordinate(double x, double y, double z)
    {
        if (!m_dashboard.IsConnected())
        {
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError = "Dashboard not connected";
            return false;
        }

        // 1. Check feedback for errors / collisions before sending.
        {
            std::lock_guard<std::mutex> lk(m_feedbackMutex);
            if (m_latestFeedback.ErrorStatus)
            {
                std::lock_guard<std::mutex> ek(m_errorMutex);
                m_lastError = "Robot error detected – refusing to send coordinate";
                return false;
            }
            if (m_latestFeedback.RobotMode == 11)
            {
                std::lock_guard<std::mutex> ek(m_errorMutex);
                m_lastError = "Collision detected (RobotMode==11) – refusing to send";
                return false;
            }
        }

        // 2. Coordinate mapping: rawCoord * scale + offset
        double mappedX = x * m_mapping.scaleX + m_mapping.offsetX;
        double mappedY = y * m_mapping.scaleY + m_mapping.offsetY;
        double mappedZ = z * m_mapping.scaleZ + m_mapping.offsetZ;

        // 3. Soft-limit clamping
        mappedX = clamp(mappedX, m_softLimits.minX, m_softLimits.maxX);
        mappedY = clamp(mappedY, m_softLimits.minY, m_softLimits.maxY);
        mappedZ = clamp(mappedZ, m_softLimits.minZ, m_softLimits.maxZ);

        // 4. Assemble CDescartesPoint with fixed orientation
        Dobot::CDescartesPoint pt;
        pt.x  = mappedX;
        pt.y  = mappedY;
        pt.z  = mappedZ;
        pt.rx = m_orientation.rx;
        pt.ry = m_orientation.ry;
        pt.rz = m_orientation.rz;

        // 5. Send ServoP via Dashboard
        std::string reply = m_dashboard.ServoP(pt);
        if (reply.find("device does not connected") != std::string::npos ||
            reply.find("send error") != std::string::npos)
        {
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError = "ServoP failed: " + reply;
            return false;
        }

        return true;
    }

    bool DobotDirectStrategy::isConnected() const
    {
        return m_dashboard.IsConnected() && m_feedback.IsConnected();
    }

    std::string DobotDirectStrategy::lastError() const
    {
        std::lock_guard<std::mutex> lk(m_errorMutex);
        return m_lastError;
    }

    void DobotDirectStrategy::setOrientation(const Orientation& o)
    {
        m_orientation = o;
    }

    void DobotDirectStrategy::setSoftLimits(const SoftLimits& limits)
    {
        m_softLimits = limits;
    }

    void DobotDirectStrategy::setCoordinateMapping(const CoordinateMapping& mapping)
    {
        m_mapping = mapping;
    }

    Dobot::CFeedbackData DobotDirectStrategy::latestFeedback() const
    {
        std::lock_guard<std::mutex> lk(m_feedbackMutex);
        return m_latestFeedback;
    }

    // -----------------------------------------------------------------------
    // Private
    // -----------------------------------------------------------------------

    void DobotDirectStrategy::feedbackLoop()
    {
        while (m_feedbackRunning.load(std::memory_order_relaxed))
        {
            if (m_feedback.IsConnected())
            {
                const Dobot::CFeedbackData& data = m_feedback.GetFeedbackData();
                std::lock_guard<std::mutex> lk(m_feedbackMutex);
                m_latestFeedback = data;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    double DobotDirectStrategy::clamp(double value, double lo, double hi)
    {
        if (value < lo) return lo;
        if (value > hi) return hi;
        return value;
    }
} // namespace touchpanel
