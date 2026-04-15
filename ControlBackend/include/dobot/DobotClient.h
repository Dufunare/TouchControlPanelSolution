#pragma once
// Ported from Dobot TCP-IP-CR-CPP-V4 official demo.
// Windows-only build: relies on WinSock2 (included via pch.h).

#include <WinSock2.h>
#include <WS2tcpip.h>

#include <string>

namespace Dobot
{
    class CDobotClient
    {
    public:
        CDobotClient();
        virtual ~CDobotClient();

        static bool InitNet();
        static void UninitNet();

        std::string GetIp() const;
        unsigned short GetPort() const;

        bool Connect(std::string strIp, unsigned short iPort);
        void Disconnect();
        bool IsConnected() const;

    protected:
        virtual void OnConnected() = 0;
        virtual void OnDisconnected() = 0;

        void Construct();
        void DestroyConstruct();

        bool SendData(std::string str);
        std::string WaitReply(int iTimeoutMillsecond);
        int Receive(char* pBuffer, int iLen);

    private:
        std::string m_strIp;
        unsigned short m_iPort = 0;
        SOCKET m_sockListen = INVALID_SOCKET;
        bool m_bIsConnect = false;
    };
} // namespace Dobot
