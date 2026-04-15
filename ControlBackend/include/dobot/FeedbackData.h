#pragma once
// Ported from Dobot TCP-IP-CR-CPP-V4 official demo.

#include <cstdint>

namespace Dobot
{
    class CFeedbackData
    {
    public:
        unsigned short MessageSize = 0;

        short Reserved1[3]{};

        int64_t DigitalInputs = 0;
        int64_t DigitalOutputs = 0;
        int64_t RobotMode = 0;
        int64_t TimeStamp = 0;
        int64_t RunTime = 0;
        int64_t TestValue = 0;

        unsigned char Reserved2[8]{};

        double SpeedScaling = 0;

        unsigned char Reserved3[16]{};

        double VRobot = 0;
        double IRobot = 0;
        double ProgramState = 0;
        char SafetyOIn[2]{};
        char SafetyOOut[2]{};

        unsigned char Reserved4[76]{};

        double QTarget[6]{};
        double QdTarget[6]{};
        double QddTarget[6]{};
        double ITarget[6]{};
        double MTarget[6]{};
        double QActual[6]{};
        double QDActual[6]{};
        double IActual[6]{};

        double ActualTCPForce[6]{};
        double ToolVectorActual[6]{};
        double ToolSpeedActual[6]{};
        double TCPForce[6]{};
        double ToolVectorTarget[6]{};
        double TCPSpeedTarget[6]{};
        double MotorTempetatures[6]{};
        double JointModes[6]{};
        double VActual[6]{};

        unsigned char Handtype[4]{};
        unsigned char User = 0;
        unsigned char Tool = 0;
        unsigned char RunQueuedCmd = 0;
        unsigned char PauseCmdFlag = 0;
        unsigned char VelocityRatio = 0;
        unsigned char AccelerationRatio = 0;
        unsigned char Reserved5 = 0;
        unsigned char XYZVelocityRatio = 0;
        unsigned char RVelocityRatio = 0;
        unsigned char XYZAccelerationRatio = 0;
        unsigned char RAccelerationRatio = 0;

        unsigned char Reserved6[2]{};

        unsigned char BrakeStatus = 0;
        unsigned char EnableStatus = 0;
        unsigned char DragStatus = 0;
        unsigned char RunningStatus = 0;
        unsigned char ErrorStatus = 0;
        unsigned char JogStatusCR = 0;
        unsigned char CRRobotType = 0;
        unsigned char DragButtonSignal = 0;
        unsigned char EnableButtonSignal = 0;
        unsigned char RecordButtonSignal = 0;
        unsigned char ReappearButtonSignal = 0;
        unsigned char JawButtonSignal = 0;
        unsigned char SixForceOnline = 0;
        unsigned char CollisionStatus = 0;
        unsigned char ArmApproachSatus = 0;
        unsigned char J4ApproachSatus = 0;
        unsigned char J5ApproachSatus = 0;
        unsigned char J6ApproachSatus = 0;

        unsigned char Reserved7[61]{};

        double VibrationDisZ = 0;
        int64_t CurrentCommandId = 0;
        double MActual[6]{};
        double Load = 0;
        double CenterX = 0;
        double CenterY = 0;
        double CenterZ = 0;
        double UserValue[6]{};
        double ToolValue[6]{};

        unsigned char Reserved8[8]{};

        double SixForceValue[6]{};
        double TargetQuaternion[4]{};
        double ActualQuaternion[4]{};
        short AutoManualMode = 0;
        unsigned short ExportStatus = 0;
        char SafetyState = 0;
        char SafetyState_Reserved = 0;
        unsigned char Reserved9[18]{};
    };
} // namespace Dobot
