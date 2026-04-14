// TouchBackendLib.cpp : 定义静态库的函数。
//

#include "pch.h"
#include "framework.h"

#include "TouchBackend.h"



namespace touchpanel
{
    namespace
    {
        std::string errorToString(const HDErrorInfo& error, const std::string& prefix)
        {
            const auto* errorText = hdGetErrorString(error.errorCode);
            if (errorText == nullptr)
            {
                return prefix + "未知 OpenHaptics 错误";
            }

            return prefix + errorText;
        }
    } // namespace

    struct TouchBackend::Impl
    {
        HHD deviceHandle{};
        HDSchedulerHandle schedulerHandle{};

        bool hasDevice = false;
        bool hasCallback = false;

        std::atomic<bool> initialized{ false };
        std::atomic<bool> running{ false };
        std::atomic<bool> valid{ false };

        std::atomic<double> x{ 0.0 };
        std::atomic<double> y{ 0.0 };
        std::atomic<double> z{ 0.0 };

        std::atomic<std::uint64_t> sampleCounter{ 0 };
        std::atomic<int> lastErrorCode{ HD_SUCCESS };

        std::string lastErrorText;

        static HDCallbackCode HDCALLBACK servoCallback(void* userData)
        {
            auto* self = static_cast<Impl*>(userData);
            if (self == nullptr || !self->running.load(std::memory_order_relaxed))
            {
                return HD_CALLBACK_DONE;
            }

            // OpenHaptics 官方要求 hdGet / hdSet 这类状态访问放在 haptics frame 内执行。
            hdBeginFrame(self->deviceHandle);

            HDdouble position[3] = { 0.0, 0.0, 0.0 };
            hdGetDoublev(HD_CURRENT_POSITION, position);

            const HDErrorInfo error = hdGetError();
            hdEndFrame(self->deviceHandle);

            if (HD_DEVICE_ERROR(error))
            {
                self->lastErrorCode.store(static_cast<int>(error.errorCode), std::memory_order_relaxed);
                self->valid.store(false, std::memory_order_relaxed);
                self->running.store(false, std::memory_order_relaxed);
                return HD_CALLBACK_DONE;
            }

            self->x.store(position[0], std::memory_order_relaxed);
            self->y.store(position[1], std::memory_order_relaxed);
            self->z.store(position[2], std::memory_order_relaxed);
            self->sampleCounter.fetch_add(1, std::memory_order_relaxed);
            self->valid.store(true, std::memory_order_relaxed);

            return HD_CALLBACK_CONTINUE;
        }
    };

    TouchBackend::TouchBackend()
        : m_impl(std::make_unique<Impl>())
    {
    }

    TouchBackend::~TouchBackend()
    {
        stop();

        if (m_impl->hasDevice)
        {
            hdDisableDevice(m_impl->deviceHandle);
            m_impl->hasDevice = false;
            m_impl->initialized.store(false, std::memory_order_relaxed);
        }
    }

    bool TouchBackend::initialize()
    {
        if (m_impl->initialized.load(std::memory_order_relaxed))
        {
            return true;
        }

        // 这里使用默认设备。后续如需多设备支持，可以把设备名或设备索引做成配置项。
        m_impl->deviceHandle = hdInitDevice(HD_DEFAULT_DEVICE);

        const HDErrorInfo error = hdGetError();
        if (HD_DEVICE_ERROR(error))
        {
            m_impl->lastErrorCode.store(static_cast<int>(error.errorCode), std::memory_order_relaxed);
            m_impl->lastErrorText = errorToString(error, "初始化 Touch 设备失败：");
            return false;
        }

        m_impl->hasDevice = true;
        m_impl->initialized.store(true, std::memory_order_relaxed);
        m_impl->lastErrorCode.store(HD_SUCCESS, std::memory_order_relaxed);
        m_impl->lastErrorText.clear();

        // 某些设备和驱动组合下，正式项目还需要加上校准流程。
        // 这一版骨架先聚焦“取坐标 + 前端显示”，你可以后续按官方示例补充校准逻辑。

        return true;
    }

    bool TouchBackend::start()
    {
        if (!m_impl->initialized.load(std::memory_order_relaxed))
        {
            m_impl->lastErrorText = "后端尚未初始化，请先调用 initialize().";
            return false;
        }

        if (m_impl->running.load(std::memory_order_relaxed))
        {
            return true;
        }

        m_impl->lastErrorText.clear();
        m_impl->lastErrorCode.store(HD_SUCCESS, std::memory_order_relaxed);
        m_impl->sampleCounter.store(0, std::memory_order_relaxed);

        m_impl->schedulerHandle =
            hdScheduleAsynchronous(&Impl::servoCallback, m_impl.get(), HD_DEFAULT_SCHEDULER_PRIORITY);

        {
            const HDErrorInfo error = hdGetError();
            if (HD_DEVICE_ERROR(error))
            {
                m_impl->lastErrorCode.store(static_cast<int>(error.errorCode), std::memory_order_relaxed);
                m_impl->lastErrorText = errorToString(error, "注册 OpenHaptics scheduler 回调失败：");
                return false;
            }
        }

        m_impl->hasCallback = true;
        m_impl->running.store(true, std::memory_order_relaxed);

        hdStartScheduler();

        {
            const HDErrorInfo error = hdGetError();
            if (HD_DEVICE_ERROR(error))
            {
                m_impl->lastErrorCode.store(static_cast<int>(error.errorCode), std::memory_order_relaxed);
                m_impl->lastErrorText = errorToString(error, "启动 OpenHaptics scheduler 失败：");

                if (m_impl->hasCallback)
                {
                    hdUnschedule(m_impl->schedulerHandle);
                    m_impl->hasCallback = false;
                }

                m_impl->running.store(false, std::memory_order_relaxed);
                return false;
            }
        }

        return true;
    }

    void TouchBackend::stop()
    {
        if (m_impl == nullptr)
        {
            return;
        }

        m_impl->running.store(false, std::memory_order_relaxed);
        m_impl->valid.store(false, std::memory_order_relaxed);

        if (m_impl->hasCallback)
        {
            hdUnschedule(m_impl->schedulerHandle);
            m_impl->hasCallback = false;
        }

        if (m_impl->initialized.load(std::memory_order_relaxed))
        {
            hdStopScheduler();
        }
    }

    DeviceState TouchBackend::latestState() const
    {
        DeviceState state;
        state.positionMm[0] = m_impl->x.load(std::memory_order_relaxed);
        state.positionMm[1] = m_impl->y.load(std::memory_order_relaxed);
        state.positionMm[2] = m_impl->z.load(std::memory_order_relaxed);
        state.initialized = m_impl->initialized.load(std::memory_order_relaxed);
        state.valid = m_impl->valid.load(std::memory_order_relaxed);
        state.schedulerRunning = m_impl->running.load(std::memory_order_relaxed);
        state.sampleCounter = m_impl->sampleCounter.load(std::memory_order_relaxed);
        return state;
    }

    bool TouchBackend::isInitialized() const
    {
        return m_impl->initialized.load(std::memory_order_relaxed);
    }

    bool TouchBackend::isRunning() const
    {
        return m_impl->running.load(std::memory_order_relaxed);
    }

    std::string TouchBackend::lastError() const
    {
        if (!m_impl->lastErrorText.empty())
        {
            return m_impl->lastErrorText;
        }

        const int errorCode = m_impl->lastErrorCode.load(std::memory_order_relaxed);
        if (errorCode == HD_SUCCESS)
        {
            return {};
        }

        const auto* errorText = hdGetErrorString(static_cast<HDerror>(errorCode));
        if (errorText == nullptr)
        {
            return "未知 OpenHaptics 错误";
        }

        return std::string(errorText);
    }
} // namespace touchpanel

