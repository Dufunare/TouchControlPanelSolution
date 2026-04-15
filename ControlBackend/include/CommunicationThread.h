#pragma once

#include "ICommunicationStrategy.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace touchpanel
{
    /// Independent network-send thread that decouples the high-frequency
    /// Touch servo loop (1000 Hz) from the much slower robot-arm communication
    /// (~33 Hz / 30 ms interval).
    ///
    /// Usage:
    ///   1. Construct with a concrete ICommunicationStrategy.
    ///   2. Call start() to launch the background sender thread.
    ///   3. From the 1000 Hz servo callback, call updateCoordinate(x,y,z).
    ///   4. The background thread wakes up every ~30 ms, reads the latest
    ///      coordinate from the thread-safe cache, and forwards it through
    ///      the active strategy.
    ///   5. Call stop() when done.
    class CommunicationThread
    {
    public:
        /// @param strategy  Ownership is NOT transferred; the caller must
        ///                   ensure the strategy outlives this object.
        explicit CommunicationThread(ICommunicationStrategy* strategy);
        ~CommunicationThread();

        CommunicationThread(const CommunicationThread&) = delete;
        CommunicationThread& operator=(const CommunicationThread&) = delete;

        /// Launch the 33 Hz sender thread.
        void start();

        /// Signal the thread to stop and join it.
        void stop();

        /// Thread-safe: write the latest Touch coordinate into the cache.
        /// Designed to be called from the 1000 Hz servo callback.
        void updateCoordinate(double x, double y, double z);

        bool isRunning() const;

        /// Change the send interval (default 30 ms ≈ 33 Hz).
        void setSendIntervalMs(unsigned int ms);

    private:
        void threadFunc();

        ICommunicationStrategy* m_strategy = nullptr;

        std::thread m_thread;
        std::atomic<bool> m_running{ false };
        unsigned int m_intervalMs = 30;

        // Thread-safe coordinate cache written by the servo loop.
        mutable std::mutex m_cacheMutex;
        double m_cacheX = 0.0;
        double m_cacheY = 0.0;
        double m_cacheZ = 0.0;
        bool m_hasNewData = false;
    };
} // namespace touchpanel
