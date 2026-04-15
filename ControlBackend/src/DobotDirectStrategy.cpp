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

    bool DobotDirectStrategy::connect(const std::string& ip, unsigned short port)
    {
        if (isConnected())
        {
            return true;
        }

        if (!Dobot::CDobotClient::InitNet())
        {
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError = "Failed to initialize WinSock";
            return false;
        }
        m_netInitialized = true;

        const unsigned short controlPort = (port == 0) ? 29999 : port;
        const unsigned short feedbackPort = static_cast<unsigned short>(controlPort + 5);

        if (!m_dashboard.Connect(ip, controlPort))
        {
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError = "Failed to connect to Dashboard (port " + std::to_string(controlPort) + ") at " + ip;
            Dobot::CDobotClient::UninitNet();
            m_netInitialized = false;
            return false;
        }

        if (!m_feedback.Connect(ip, feedbackPort))
        {
            m_dashboard.Disconnect();
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError = "Failed to connect to Feedback (port " + std::to_string(feedbackPort) + ") at " + ip;
            Dobot::CDobotClient::UninitNet();
            m_netInitialized = false;
            return false;
        }

        // Start the local feedback polling thread.
        m_feedbackRunning.store(true, std::memory_order_release);
        m_feedbackThread = std::thread(&DobotDirectStrategy::feedbackLoop, this);

        {
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError.clear();
        }

        return true;
    }

    void DobotDirectStrategy::disconnect()
    {
        m_feedbackRunning.store(false, std::memory_order_release);
        if (m_feedbackThread.joinable())
        {
            m_feedbackThread.join();
        }

        m_dashboard.Disconnect();
        m_feedback.Disconnect();

        if (m_netInitialized)
        {
            Dobot::CDobotClient::UninitNet();
            m_netInitialized = false;
        }
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
        bool hasRobotError = false;
        bool isCollisionMode = false;
        {
            std::lock_guard<std::mutex> lk(m_feedbackMutex);
            hasRobotError = (m_latestFeedback.ErrorStatus != 0);
            isCollisionMode = (m_latestFeedback.RobotMode == 11);
        }

        if (hasRobotError)
        {
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError = "Robot error detected – refusing to send coordinate";
            return false;
        }

        if (isCollisionMode)
        {
            std::lock_guard<std::mutex> lk(m_errorMutex);
            m_lastError = "Collision detected (RobotMode==11) – refusing to send";
            return false;
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
        while (m_feedbackRunning.load(std::memory_order_acquire))
        {
            if (m_feedback.IsConnected())
            {
                const Dobot::CFeedbackData data = m_feedback.GetFeedbackData();
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
