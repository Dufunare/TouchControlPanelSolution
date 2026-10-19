// TouchBackendLib.cpp : 定义静态库的函数。
//

#include "pch.h"
#include "framework.h"

#include "TouchBackend.h"
#include "TeleopGeometry.h"
#include "HapticCalibrationPolicy.h"
#include <cstdio>



namespace touchpanel
{
    namespace
    {
        std::string errorToString(const HDErrorInfo& error, const std::string& prefix)
        {
            const auto* errorText = hdGetErrorString(error.errorCode);
            if (errorText == nullptr)
            {
                return prefix + "Unknown OpenHaptics error";
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
        std::array<std::atomic<double>, 9> rotation{};
        std::atomic<bool> orientationValid{false};
        std::atomic<std::uint64_t> frameVersion{0}; // odd=writer in progress
        std::atomic<bool> calibrated{false};
        std::atomic<bool> calibrationNeedsManual{true};
        std::atomic<std::uint64_t> calibrationEpoch{0};
        HDint calibrationStyle = 0;
        // Only touched by the OpenHaptics scheduler thread, never by Qt.
        haptics::CalibrationTickPolicy calibrationPolicy;
        std::atomic<std::uint64_t> diagnosticTicks{0};
        std::atomic<std::uint64_t> diagnosticUpdates{0};
        std::atomic<int> failureStage{0};
        std::atomic<int> failureCode{HD_SUCCESS};
        bool schedulerStarted = false;  // owned by the UI thread
        std::atomic<bool> button1Pressed{ false };

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

            using haptics::CalibrationAction;
            using haptics::CalibrationCondition;
            const auto action=self->calibrationPolicy.next();
            self->diagnosticTicks.store(self->calibrationPolicy.ticks(),std::memory_order_relaxed);
            if(action==CalibrationAction::CheckOnly) {
                hdBeginFrame(self->deviceHandle);
                const HDenum status=hdCheckCalibration();
                hdEndFrame(self->deviceHandle);
                const HDErrorInfo error=hdGetError();
                if(HD_DEVICE_ERROR(error)) {
                    self->failureStage.store(1,std::memory_order_relaxed);
                    self->failureCode.store(static_cast<int>(error.errorCode),std::memory_order_relaxed);
                    self->lastErrorCode.store(static_cast<int>(error.errorCode),std::memory_order_relaxed);
                    self->valid.store(false,std::memory_order_relaxed);
                    self->running.store(false,std::memory_order_relaxed);
                    return HD_CALLBACK_DONE;
                }
                const bool ready=status==HD_CALIBRATION_OK;
                const bool previouslyReady=self->calibrated.exchange(ready,std::memory_order_relaxed);
                self->calibrationNeedsManual.store(status==HD_CALIBRATION_NEEDS_MANUAL_INPUT,std::memory_order_relaxed);
                if(ready!=previouslyReady)
                    self->calibrationEpoch.fetch_add(1,std::memory_order_relaxed);
                self->calibrationPolicy.checked(
                    status==HD_CALIBRATION_NEEDS_UPDATE?CalibrationCondition::NeedsUpdate:
                    status==HD_CALIBRATION_NEEDS_MANUAL_INPUT?CalibrationCondition::ManualInput:
                    ready?CalibrationCondition::Good:CalibrationCondition::Unsupported,
                    self->calibrationStyle!=0);
                if(!ready)self->valid.store(false,std::memory_order_relaxed);
                return HD_CALLBACK_CONTINUE;
            }
            if(action==CalibrationAction::UpdateOnly) {
                hdMakeCurrentDevice(self->deviceHandle);
                hdUpdateCalibration(self->calibrationStyle);
                const HDErrorInfo error=hdGetError();
                if(HD_DEVICE_ERROR(error)) {
                    self->failureStage.store(2,std::memory_order_relaxed);
                    self->failureCode.store(static_cast<int>(error.errorCode),std::memory_order_relaxed);
                    self->lastErrorCode.store(static_cast<int>(error.errorCode),std::memory_order_relaxed);
                    self->valid.store(false,std::memory_order_relaxed);
                    self->running.store(false,std::memory_order_relaxed);
                    return HD_CALLBACK_DONE;
                }
                self->calibrationPolicy.updated();
                self->diagnosticUpdates.store(self->calibrationPolicy.updateCount(),std::memory_order_relaxed);
                self->calibrated.store(false,std::memory_order_relaxed);
                self->calibrationEpoch.fetch_add(1,std::memory_order_relaxed);
                self->valid.store(false,std::memory_order_relaxed);
                return HD_CALLBACK_CONTINUE;
            }

            // A normal tick contains exactly one begin/end pair. Calibration
            // check frames and updates execute in distinct scheduler ticks.
            hdBeginFrame(self->deviceHandle);
            HDdouble position[3] = { 0.0, 0.0, 0.0 };
            HDint currentButtons = 0;
            HDdouble transform[16]{};
            hdGetDoublev(HD_CURRENT_POSITION, position);
            hdGetIntegerv(HD_CURRENT_BUTTONS, &currentButtons);
            hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
            hdEndFrame(self->deviceHandle);
            const HDErrorInfo error=hdGetError();
            if(HD_DEVICE_ERROR(error)) {
                self->failureStage.store(3,std::memory_order_relaxed);
                self->failureCode.store(static_cast<int>(error.errorCode),std::memory_order_relaxed);
                self->lastErrorCode.store(static_cast<int>(error.errorCode),std::memory_order_relaxed);
                self->valid.store(false,std::memory_order_relaxed);
                self->running.store(false,std::memory_order_relaxed);
                return HD_CALLBACK_DONE;
            }
            if(!self->calibrated.load(std::memory_order_relaxed)) {
                self->valid.store(false,std::memory_order_relaxed);
                return HD_CALLBACK_CONTINUE;
            }

            self->frameVersion.fetch_add(1,std::memory_order_release);
            self->x.store(position[0], std::memory_order_relaxed);
            self->y.store(position[1], std::memory_order_relaxed);
            self->z.store(position[2], std::memory_order_relaxed);
            std::array<double,16> matrix{};
            for(int i=0;i<16;++i)matrix[i]=transform[i];
            const auto orientation=teleop::openHapticsRotation(matrix);
            const bool orientationOk=teleop::properRotation(orientation);
            if(orientationOk)for(int i=0;i<9;++i)self->rotation[i].store(orientation[i],std::memory_order_relaxed);
            self->orientationValid.store(orientationOk,std::memory_order_relaxed);
            self->button1Pressed.store((currentButtons & HD_DEVICE_BUTTON_1) != 0, std::memory_order_relaxed);
            self->sampleCounter.fetch_add(1, std::memory_order_relaxed);
            self->valid.store(self->calibrated.load(std::memory_order_relaxed) && orientationOk,
                std::memory_order_relaxed);
            self->frameVersion.fetch_add(1,std::memory_order_release);

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

        m_impl->running.store(false, std::memory_order_relaxed);
        m_impl->valid.store(false, std::memory_order_relaxed);

        if (m_impl->hasDevice && m_impl->deviceHandle != HD_INVALID_HANDLE)
        {
            hdDisableDevice(m_impl->deviceHandle);
        }
        m_impl->deviceHandle = HD_INVALID_HANDLE;
        m_impl->hasDevice = false;
        m_impl->initialized.store(false, std::memory_order_relaxed);

        HDErrorInfo error{};
        while (HD_DEVICE_ERROR(error = hdGetError()))
        {
        }

        // 这里使用默认设备。后续如需多设备支持，可以把设备名或设备索引做成配置项。
        m_impl->deviceHandle = hdInitDevice(HD_DEFAULT_DEVICE);

        error = hdGetError();
        if (m_impl->deviceHandle == HD_INVALID_HANDLE || HD_DEVICE_ERROR(error))
        {
            if (m_impl->deviceHandle != HD_INVALID_HANDLE)
            {
                hdDisableDevice(m_impl->deviceHandle);
                m_impl->deviceHandle = HD_INVALID_HANDLE;
            }

            m_impl->hasDevice = false;
            m_impl->initialized.store(false, std::memory_order_relaxed);
            m_impl->running.store(false, std::memory_order_relaxed);
            m_impl->valid.store(false, std::memory_order_relaxed);

            if (HD_DEVICE_ERROR(error))
            {
                m_impl->lastErrorCode.store(static_cast<int>(error.errorCode), std::memory_order_relaxed);
                m_impl->lastErrorText = errorToString(error, "设备初始化失败，详细信息\n");
            }
            else
            {
                m_impl->lastErrorCode.store(-1, std::memory_order_relaxed);
                m_impl->lastErrorText = "初始化失败，没有找到设备或驱动状态异常，请检查设备连接";
            }

            return false;
        }

        m_impl->hasDevice = true;
        m_impl->initialized.store(true, std::memory_order_relaxed);
        m_impl->lastErrorCode.store(HD_SUCCESS, std::memory_order_relaxed);
        m_impl->lastErrorText.clear();
        m_impl->calibrationStyle = 0;
        m_impl->calibrationPolicy.reset();

        HDint styles=0;
        hdGetIntegerv(HD_CALIBRATION_STYLE,&styles);
        if(styles & HD_CALIBRATION_INKWELL) m_impl->calibrationStyle=HD_CALIBRATION_INKWELL;
        else if(styles & HD_CALIBRATION_AUTO) m_impl->calibrationStyle=HD_CALIBRATION_AUTO;
        else if(styles & HD_CALIBRATION_ENCODER_RESET) m_impl->calibrationStyle=HD_CALIBRATION_ENCODER_RESET;
        m_impl->calibrated.store(false,std::memory_order_relaxed);
        m_impl->calibrationNeedsManual.store(true,std::memory_order_relaxed);
        // 某些设备和驱动组合下，正式项目还需要加上校准流程。
        // 这一版骨架先聚焦“取坐标 + 前端显示”，你可以后续按官方示例补充校准逻辑。

        return true;
    }

    bool TouchBackend::start()
    {
        if (!m_impl->initialized.load(std::memory_order_relaxed))
        {
            m_impl->lastErrorText = "未初始化，请先调用 initialize()。";
            return false;
        }

        if (m_impl->running.load(std::memory_order_relaxed))
        {
            return true;
        }

        // A callback that returned HD_CALLBACK_DONE on an earlier error can
        // leave the scheduler/device owned by this object. Reclaim it before
        // registering a new callback, including after an aborted start.
        if (m_impl->hasCallback || m_impl->schedulerStarted)
            stop();

        // Cleanup can leave a stale HD error in the queue. Keep the previous
        // failure in lastErrorText, then start with a clear HDAPI error queue.
        while (HD_DEVICE_ERROR(hdGetError())) { }
        m_impl->lastErrorText.clear();
        m_impl->lastErrorCode.store(HD_SUCCESS, std::memory_order_relaxed);
        m_impl->sampleCounter.store(0, std::memory_order_relaxed);
        m_impl->calibrationPolicy.reset();
        m_impl->diagnosticTicks.store(0,std::memory_order_relaxed);
        m_impl->diagnosticUpdates.store(0,std::memory_order_relaxed);
        m_impl->failureStage.store(0,std::memory_order_relaxed);
        m_impl->failureCode.store(HD_SUCCESS,std::memory_order_relaxed);
        m_impl->calibrated.store(false, std::memory_order_relaxed);
        m_impl->valid.store(false, std::memory_order_relaxed);

        m_impl->schedulerHandle =
            hdScheduleAsynchronous(&Impl::servoCallback, m_impl.get(), HD_DEFAULT_SCHEDULER_PRIORITY);

        {
            const HDErrorInfo error = hdGetError();
            if (HD_DEVICE_ERROR(error))
            {
                m_impl->lastErrorCode.store(static_cast<int>(error.errorCode), std::memory_order_relaxed);
                m_impl->lastErrorText = errorToString(error, "注册 OpenHaptics 调度器失败，详细信息\n");
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
                m_impl->lastErrorText = errorToString(error, "开始调度器失败：");

                // Stop BEFORE unscheduling, per OpenHaptics cleanup guidance.
                // Avoid leaving a scheduled callback behind on startup failure.
                m_impl->running.store(false, std::memory_order_relaxed);
                hdStopScheduler();
                hdUnschedule(m_impl->schedulerHandle);
                m_impl->hasCallback = false;
                m_impl->schedulerStarted = false;
                while (HD_DEVICE_ERROR(hdGetError())) { }
                return false;
            }
        }

        m_impl->schedulerStarted = true;
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

        // The documentation's cleanup order is stop scheduler first, then
        // remove registered operations. Do not call hdStopScheduler again on
        // an already-stopped scheduler.
        if (m_impl->schedulerStarted)
        {
            hdStopScheduler();
            m_impl->schedulerStarted = false;
        }
        if (m_impl->hasCallback)
        {
            hdUnschedule(m_impl->schedulerHandle);
            m_impl->hasCallback = false;
        }
        m_impl->calibrated.store(false, std::memory_order_relaxed);
        m_impl->calibrationNeedsManual.store(true, std::memory_order_relaxed);
        m_impl->calibrationEpoch.fetch_add(1, std::memory_order_relaxed);
    }

    void TouchBackend::reset()
    {
        stop();

        if (m_impl->hasDevice)
        {
            hdDisableDevice(m_impl->deviceHandle);
            m_impl->hasDevice = false;
            m_impl->initialized.store(false, std::memory_order_relaxed);
        }

        m_impl->lastErrorCode.store(HD_SUCCESS, std::memory_order_relaxed);
        m_impl->lastErrorText.clear();
        m_impl->sampleCounter.store(0, std::memory_order_relaxed);
        m_impl->x.store(0.0, std::memory_order_relaxed);
        m_impl->y.store(0.0, std::memory_order_relaxed);
        m_impl->z.store(0.0, std::memory_order_relaxed);
        m_impl->button1Pressed.store(false, std::memory_order_relaxed);
        m_impl->orientationValid.store(false, std::memory_order_relaxed);
        m_impl->calibrated.store(false, std::memory_order_relaxed);
        m_impl->calibrationEpoch.fetch_add(1,std::memory_order_relaxed);
    }

    DeviceState TouchBackend::latestState() const
    {
        DeviceState state;
        // All fields are atomics; validate one complete publication version.
        // Bounded retries prevent UI stalls during an interrupted device callback.
        for (int retry=0;retry<8;++retry) {
            const auto before=m_impl->frameVersion.load(std::memory_order_acquire);
            if(before & 1u) continue;
            for(int i=0;i<3;++i)state.positionMm[i]=
                (i==0?m_impl->x.load(std::memory_order_relaxed):
                 i==1?m_impl->y.load(std::memory_order_relaxed):
                      m_impl->z.load(std::memory_order_relaxed));
            for(int i=0;i<9;++i)state.rotationMatrix[i]=m_impl->rotation[i].load(std::memory_order_relaxed);
            state.orientationValid=m_impl->orientationValid.load(std::memory_order_relaxed);
            state.button1Pressed=m_impl->button1Pressed.load(std::memory_order_relaxed);
            state.sampleCounter=m_impl->sampleCounter.load(std::memory_order_relaxed);
            state.valid=m_impl->valid.load(std::memory_order_relaxed);
            const auto after=m_impl->frameVersion.load(std::memory_order_acquire);
            if(before==after && !(after & 1u)) {
                state.initialized=m_impl->initialized.load(std::memory_order_relaxed);
                state.schedulerRunning=m_impl->running.load(std::memory_order_relaxed);
                state.deviceCalibrated=m_impl->calibrated.load(std::memory_order_relaxed);
                state.calibrationNeedsManualInput=m_impl->calibrationNeedsManual.load(std::memory_order_relaxed);
                state.calibrationEpoch=m_impl->calibrationEpoch.load(std::memory_order_relaxed);
                state.valid=state.valid && state.deviceCalibrated && state.schedulerRunning;
                return state;
            }
        }
        // Retry exhaustion means ONLY that this particular multi-atomic sample
        // was incoherent, not that the OpenHaptics scheduler stopped. Always
        // load lifecycle flags independently of the versioned sample. A default
        // DeviceState would report schedulerRunning=false and cause Qt to stop
        // polling despite a healthy 1 kHz servo thread.
        state.valid=false;
        state.initialized=m_impl->initialized.load(std::memory_order_relaxed);
        state.schedulerRunning=m_impl->running.load(std::memory_order_relaxed);
        state.sampleCounter=m_impl->sampleCounter.load(std::memory_order_relaxed);
        state.deviceCalibrated=m_impl->calibrated.load(std::memory_order_relaxed);
        state.calibrationNeedsManualInput=m_impl->calibrationNeedsManual.load(std::memory_order_relaxed);
        state.calibrationEpoch=m_impl->calibrationEpoch.load(std::memory_order_relaxed);
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
        const int stage=m_impl->failureStage.load(std::memory_order_relaxed);
        const char* stageName=stage==1?"calibration-check":stage==2?"calibration-update":
            stage==3?"sample-frame":"startup/other";
        if (errorCode == HD_SUCCESS)
        {
            return {};
        }

        const auto* errorText = hdGetErrorString(static_cast<HDerror>(errorCode));
        if (errorText == nullptr)
        {
            return "Unknown OpenHaptics Error";
        }

        return std::string("HDAPI stage=")+stageName+" code="+std::to_string(errorCode)+
               " ticks="+std::to_string(m_impl->diagnosticTicks.load(std::memory_order_relaxed))+
               " updates="+std::to_string(m_impl->diagnosticUpdates.load(std::memory_order_relaxed))+
               ": "+errorText;
    }
} // namespace touchpanel

