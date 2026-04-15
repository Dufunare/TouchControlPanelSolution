// Ported from Dobot TCP-IP-CR-CPP-V4 official demo.
#include "pch.h"
#include "dobot/Feedback.h"
#include "dobot/BitConverter.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

namespace Dobot
{
    CFeedback::CFeedback() = default;

    CFeedback::~CFeedback()
    {
        Disconnect();
        if (m_thd.joinable())
        {
            m_thd.join();
        }
    }

    void CFeedback::OnConnected()
    {
        if (m_thd.joinable())
        {
            m_thd.join();
        }
        std::thread thd([this] { OnRecvData(); });
        m_thd.swap(thd);
    }

    void CFeedback::OnDisconnected()
    {
        if (m_thd.joinable())
        {
            if (m_thd.get_id() == std::this_thread::get_id())
            {
                m_thd.detach();
            }
            else
            {
                m_thd.join();
            }
        }
    }

    CFeedbackData CFeedback::GetFeedbackData() const
    {
        std::lock_guard<std::mutex> lk(m_dataMutex);
        return m_feedbackData;
    }

    void CFeedback::OnRecvData()
    {
        constexpr int BUFFERSIZE = 4320; // 1440 * 3
        char buffer[BUFFERSIZE] = { 0 };
        int iHasRead = 0;
        while (IsConnected())
        {
            int iRet = Receive(buffer + iHasRead, BUFFERSIZE - iHasRead);
            if (iRet <= 0)
            {
                continue;
            }
            iHasRead += iRet;
            if (iHasRead < 1440)
            {
                continue;
            }

            bool bHasFound = false;
            for (int i = 0; i < iHasRead; ++i)
            {
                unsigned short iMsgSize = CBitConverter::ToUShort(buffer + i);
                if (1440 != iMsgSize)
                {
                    continue;
                }
                uint64_t checkValue = CBitConverter::ToUInt64(buffer + i + 48);
                if (0x0123456789ABCDEF == checkValue)
                {
                    bHasFound = true;
                    if (i != 0)
                    {
                        iHasRead = iHasRead - i;
                        memmove(buffer, buffer + i, BUFFERSIZE - i);
                    }
                    break;
                }
            }
            if (!bHasFound)
            {
                if (iHasRead >= BUFFERSIZE)
                {
                    iHasRead = 0;
                }
                continue;
            }
            if (iHasRead < 1440)
            {
                continue;
            }
            iHasRead = iHasRead - 1440;
            ParseData(buffer);
            memmove(buffer, buffer + 1440, BUFFERSIZE - 1440);
        }
    }

    void CFeedback::ParseData(char* pBuffer)
    {
        std::lock_guard<std::mutex> lk(m_dataMutex);
        int iStartIndex = 0;

        m_feedbackData.MessageSize = CBitConverter::ToUShort(pBuffer + iStartIndex);
        iStartIndex += 2;

        int iArrLength = sizeof(m_feedbackData.Reserved1) / sizeof(m_feedbackData.Reserved1[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.Reserved1[i] = CBitConverter::ToShort(pBuffer + iStartIndex);
            iStartIndex += 2;
        }

        m_feedbackData.DigitalInputs  = CBitConverter::ToInt64(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.DigitalOutputs = CBitConverter::ToInt64(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.RobotMode      = CBitConverter::ToInt64(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.TimeStamp      = CBitConverter::ToInt64(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.RunTime        = CBitConverter::ToInt64(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.TestValue      = CBitConverter::ToInt64(pBuffer + iStartIndex); iStartIndex += 8;

        iArrLength = sizeof(m_feedbackData.Reserved2) / sizeof(m_feedbackData.Reserved2[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.Reserved2[i] = pBuffer[iStartIndex];
            iStartIndex += 1;
        }

        m_feedbackData.SpeedScaling = CBitConverter::ToDouble(pBuffer + iStartIndex); iStartIndex += 8;

        iArrLength = sizeof(m_feedbackData.Reserved3) / sizeof(m_feedbackData.Reserved3[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.Reserved3[i] = pBuffer[iStartIndex];
            iStartIndex += 1;
        }

        m_feedbackData.VRobot       = CBitConverter::ToDouble(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.IRobot       = CBitConverter::ToDouble(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.ProgramState = CBitConverter::ToDouble(pBuffer + iStartIndex); iStartIndex += 8;

        iArrLength = sizeof(m_feedbackData.SafetyOIn) / sizeof(m_feedbackData.SafetyOIn[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.SafetyOIn[i] = pBuffer[iStartIndex];
            iStartIndex += 1;
        }
        iArrLength = sizeof(m_feedbackData.SafetyOOut) / sizeof(m_feedbackData.SafetyOOut[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.SafetyOOut[i] = pBuffer[iStartIndex];
            iStartIndex += 1;
        }

        iArrLength = sizeof(m_feedbackData.Reserved4) / sizeof(m_feedbackData.Reserved4[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.Reserved4[i] = pBuffer[iStartIndex];
            iStartIndex += 1;
        }

        auto parseDoubleArray = [&](double* arr, int count)
        {
            for (int i = 0; i < count; ++i)
            {
                arr[i] = CBitConverter::ToDouble(pBuffer + iStartIndex);
                iStartIndex += 8;
            }
        };

        parseDoubleArray(m_feedbackData.QTarget, 6);
        parseDoubleArray(m_feedbackData.QdTarget, 6);
        parseDoubleArray(m_feedbackData.QddTarget, 6);
        parseDoubleArray(m_feedbackData.ITarget, 6);
        parseDoubleArray(m_feedbackData.MTarget, 6);
        parseDoubleArray(m_feedbackData.QActual, 6);
        parseDoubleArray(m_feedbackData.QDActual, 6);
        parseDoubleArray(m_feedbackData.IActual, 6);

        parseDoubleArray(m_feedbackData.ActualTCPForce, 6);
        parseDoubleArray(m_feedbackData.ToolVectorActual, 6);
        parseDoubleArray(m_feedbackData.ToolSpeedActual, 6);
        parseDoubleArray(m_feedbackData.TCPForce, 6);
        parseDoubleArray(m_feedbackData.ToolVectorTarget, 6);
        parseDoubleArray(m_feedbackData.TCPSpeedTarget, 6);
        parseDoubleArray(m_feedbackData.MotorTempetatures, 6);
        parseDoubleArray(m_feedbackData.JointModes, 6);
        parseDoubleArray(m_feedbackData.VActual, 6);

        iArrLength = sizeof(m_feedbackData.Handtype) / sizeof(m_feedbackData.Handtype[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.Handtype[i] = pBuffer[iStartIndex];
            iStartIndex += 1;
        }

        m_feedbackData.User                = pBuffer[iStartIndex++];
        m_feedbackData.Tool                = pBuffer[iStartIndex++];
        m_feedbackData.RunQueuedCmd        = pBuffer[iStartIndex++];
        m_feedbackData.PauseCmdFlag        = pBuffer[iStartIndex++];
        m_feedbackData.VelocityRatio       = pBuffer[iStartIndex++];
        m_feedbackData.AccelerationRatio   = pBuffer[iStartIndex++];
        m_feedbackData.Reserved5           = pBuffer[iStartIndex++];
        m_feedbackData.XYZVelocityRatio    = pBuffer[iStartIndex++];
        m_feedbackData.RVelocityRatio      = pBuffer[iStartIndex++];
        m_feedbackData.XYZAccelerationRatio = pBuffer[iStartIndex++];
        m_feedbackData.RAccelerationRatio  = pBuffer[iStartIndex++];

        iArrLength = sizeof(m_feedbackData.Reserved6) / sizeof(m_feedbackData.Reserved6[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.Reserved6[i] = pBuffer[iStartIndex];
            iStartIndex += 1;
        }

        m_feedbackData.BrakeStatus          = pBuffer[iStartIndex++];
        m_feedbackData.EnableStatus          = pBuffer[iStartIndex++];
        m_feedbackData.DragStatus            = pBuffer[iStartIndex++];
        m_feedbackData.RunningStatus         = pBuffer[iStartIndex++];
        m_feedbackData.ErrorStatus           = pBuffer[iStartIndex++];
        m_feedbackData.JogStatusCR           = pBuffer[iStartIndex++];
        m_feedbackData.CRRobotType           = pBuffer[iStartIndex++];
        m_feedbackData.DragButtonSignal      = pBuffer[iStartIndex++];
        m_feedbackData.EnableButtonSignal    = pBuffer[iStartIndex++];
        m_feedbackData.RecordButtonSignal    = pBuffer[iStartIndex++];
        m_feedbackData.ReappearButtonSignal  = pBuffer[iStartIndex++];
        m_feedbackData.JawButtonSignal       = pBuffer[iStartIndex++];
        m_feedbackData.SixForceOnline        = pBuffer[iStartIndex++];
        m_feedbackData.CollisionStatus       = pBuffer[iStartIndex++];
        m_feedbackData.ArmApproachSatus      = pBuffer[iStartIndex++];
        m_feedbackData.J4ApproachSatus       = pBuffer[iStartIndex++];
        m_feedbackData.J5ApproachSatus       = pBuffer[iStartIndex++];
        m_feedbackData.J6ApproachSatus       = pBuffer[iStartIndex++];

        iArrLength = sizeof(m_feedbackData.Reserved7) / sizeof(m_feedbackData.Reserved7[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.Reserved7[i] = pBuffer[iStartIndex];
            iStartIndex += 1;
        }

        m_feedbackData.VibrationDisZ    = CBitConverter::ToDouble(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.CurrentCommandId = CBitConverter::ToInt64(pBuffer + iStartIndex);  iStartIndex += 8;

        parseDoubleArray(m_feedbackData.MActual, 6);

        m_feedbackData.Load    = CBitConverter::ToDouble(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.CenterX = CBitConverter::ToDouble(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.CenterY = CBitConverter::ToDouble(pBuffer + iStartIndex); iStartIndex += 8;
        m_feedbackData.CenterZ = CBitConverter::ToDouble(pBuffer + iStartIndex); iStartIndex += 8;

        parseDoubleArray(m_feedbackData.UserValue, 6);
        parseDoubleArray(m_feedbackData.ToolValue, 6);

        iArrLength = sizeof(m_feedbackData.Reserved8) / sizeof(m_feedbackData.Reserved8[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.Reserved8[i] = pBuffer[iStartIndex];
            iStartIndex += 1;
        }

        parseDoubleArray(m_feedbackData.SixForceValue, 6);
        parseDoubleArray(m_feedbackData.TargetQuaternion, 4);
        parseDoubleArray(m_feedbackData.ActualQuaternion, 4);

        m_feedbackData.AutoManualMode       = CBitConverter::ToShort(pBuffer + iStartIndex);  iStartIndex += 2;
        m_feedbackData.ExportStatus          = CBitConverter::ToUShort(pBuffer + iStartIndex); iStartIndex += 2;
        m_feedbackData.SafetyState           = pBuffer[iStartIndex++];
        m_feedbackData.SafetyState_Reserved = pBuffer[iStartIndex++];

        iArrLength = sizeof(m_feedbackData.Reserved9) / sizeof(m_feedbackData.Reserved9[0]);
        for (int i = 0; i < iArrLength; ++i)
        {
            m_feedbackData.Reserved9[i] = pBuffer[iStartIndex];
            iStartIndex += 1;
        }

        m_IsDataHasRead.store(true, std::memory_order_relaxed);
    }

    std::string CFeedback::ConvertRobotMode()
    {
        switch (m_feedbackData.RobotMode)
        {
        case -1: return "NO_CONTROLLER";
        case  0: return "NO_CONNECTED";
        case  1: return "ROBOT_MODE_INIT";
        case  2: return "ROBOT_MODE_BRAKE_OPEN";
        case  3: return "ROBOT_RESERVED";
        case  4: return "ROBOT_MODE_DISABLED";
        case  5: return "ROBOT_MODE_ENABLE";
        case  6: return "ROBOT_MODE_BACKDRIVE";
        case  7: return "ROBOT_MODE_RUNNING";
        case  8: return "ROBOT_MODE_RECORDING";
        case  9: return "ROBOT_MODE_ERROR";
        case 10: return "ROBOT_MODE_PAUSE";
        case 11: return "ROBOT_MODE_JOG";
        default: break;
        }
        std::string str("UNKNOWN: RobotMode=");
        str += std::to_string(m_feedbackData.RobotMode);
        return str;
    }
} // namespace Dobot
