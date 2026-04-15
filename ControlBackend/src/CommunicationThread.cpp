#include "pch.h"
#include "CommunicationThread.h"

#include <algorithm>
#include <chrono>

namespace touchpanel
{
    CommunicationThread::CommunicationThread(ICommunicationStrategy* strategy)
        : m_strategy(strategy)
    {
    }

    CommunicationThread::~CommunicationThread()
    {
        stop();
    }

    void CommunicationThread::start()
    {
        if (m_running.load(std::memory_order_relaxed))
        {
            return;
        }

        m_running.store(true, std::memory_order_relaxed);
        m_thread = std::thread(&CommunicationThread::threadFunc, this);
    }

    void CommunicationThread::stop()
    {
        m_running.store(false, std::memory_order_relaxed);
        if (m_thread.joinable())
        {
            m_thread.join();
        }
    }

    void CommunicationThread::updateCoordinate(double x, double y, double z)
    {
        std::lock_guard<std::mutex> lk(m_cacheMutex);
        m_cacheX = x;
        m_cacheY = y;
        m_cacheZ = z;
        m_hasNewData = true;
    }

    bool CommunicationThread::isRunning() const
    {
        return m_running.load(std::memory_order_relaxed);
    }

    void CommunicationThread::setSendIntervalMs(unsigned int ms)
    {
    m_intervalMs.store((std::max)(1u, ms), std::memory_order_relaxed);
    }

    // -----------------------------------------------------------------------
    // Private
    // -----------------------------------------------------------------------

    void CommunicationThread::threadFunc()
    {
        while (m_running.load(std::memory_order_relaxed))
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            bool shouldSend = false;

            {
                std::lock_guard<std::mutex> lk(m_cacheMutex);
                if (m_hasNewData)
                {
                    x = m_cacheX;
                    y = m_cacheY;
                    z = m_cacheZ;
                    shouldSend = true;
                    m_hasNewData = false;
                }
            }

            if (shouldSend && m_strategy != nullptr)
            {
                m_strategy->sendCoordinate(x, y, z);
            }

            const unsigned int interval = m_intervalMs.load(std::memory_order_relaxed);
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
        }
    }
} // namespace touchpanel
