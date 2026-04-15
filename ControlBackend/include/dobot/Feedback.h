#pragma once
// Ported from Dobot TCP-IP-CR-CPP-V4 official demo.

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

        const CFeedbackData& GetFeedbackData() const;

        inline bool IsDataHasRead() const
        {
            return m_IsDataHasRead;
        }

        inline void SetDataHasRead(bool bValue)
        {
            m_IsDataHasRead = bValue;
        }

        std::string ConvertRobotMode();

    protected:
        void OnConnected() override;
        void OnDisconnected() override;

        void OnRecvData();
        void ParseData(char* pBuffer);

    private:
        std::thread m_thd;
        bool m_IsDataHasRead = false;
        CFeedbackData m_feedbackData;
    };
} // namespace Dobot
