#pragma once

#include <memory>
#include <string>

#include "DeviceState.h"

namespace touchpanel
{
    class TouchBackend
    {
    public:
        TouchBackend();
        ~TouchBackend();

        TouchBackend(const TouchBackend&) = delete;
        TouchBackend& operator=(const TouchBackend&) = delete;

        bool initialize();
        bool start();
        void stop();
        void reset();

        DeviceState latestState() const;

        bool isInitialized() const;
        bool isRunning() const;
        std::string lastError() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
} // namespace touchpanel
