#pragma once
// Ported from Dobot TCP-IP-CR-CPP-V4 official demo.

#include <atomic>
#include <mutex>
#include <thread>
#include "DobotClient.h"
#include "FeedbackData.h"

namespace Dobot
{
    class CFeedback : public CDobotClient
    {
    public:
        CFeedback();
        virtual ~CFeedback();

        CFeedbackData GetFeedbackData() const;

        inline bool IsDataHasRead() const
        {
            return m_IsDataHasRead.load(std::memory_order_relaxed);
        }

        inline void SetDataHasRead(bool bValue)
        {
            m_IsDataHasRead.store(bValue, std::memory_order_relaxed);
        }

        std::string ConvertRobotMode();

    protected:
        void OnConnected() override;
        void OnDisconnected() override;

        void OnRecvData();
        void ParseData(char* pBuffer);

    private:
        std::thread m_thd;
        std::atomic<bool> m_IsDataHasRead{ false };
        mutable std::mutex m_dataMutex;
        CFeedbackData m_feedbackData;
    };
} // namespace Dobot
