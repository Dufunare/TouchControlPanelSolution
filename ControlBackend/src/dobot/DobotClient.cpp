// Ported from Dobot TCP-IP-CR-CPP-V4 official demo.
#include "pch.h"
#include "dobot/DobotClient.h"

#include <iostream>

namespace Dobot
{
    std::mutex CDobotClient::s_netMutex;
    unsigned int CDobotClient::s_netRefCount = 0;
    bool CDobotClient::s_netInitialized = false;

    CDobotClient::CDobotClient()
    {
        Construct();
    }

    CDobotClient::~CDobotClient()
    {
        DestroyConstruct();
    }

    bool CDobotClient::InitNet()
    {
        std::lock_guard<std::mutex> lk(s_netMutex);

        if (s_netInitialized)
        {
            ++s_netRefCount;
            return true;
        }

        WORD wVer = MAKEWORD(2, 2);
        WSADATA wsd;
        if (0 != WSAStartup(wVer, &wsd))
        {
            std::cout << "WSAStartup fail, errcode:" << WSAGetLastError() << std::endl;
            return false;
        }
        if (2 != LOBYTE(wVer) || 2 != HIBYTE(wVer))
        {
            std::cout << "winsock is not match version\n";
            WSACleanup();
            return false;
        }

        s_netInitialized = true;
        s_netRefCount = 1;
        return true;
    }

    void CDobotClient::UninitNet()
    {
        std::lock_guard<std::mutex> lk(s_netMutex);

        if (!s_netInitialized || s_netRefCount == 0)
        {
            return;
        }

        --s_netRefCount;
        if (s_netRefCount == 0)
        {
            WSACleanup();
            s_netInitialized = false;
        }
    }

    void CDobotClient::Construct()
    {
        m_sockListen = INVALID_SOCKET;
        m_iPort = 0;
        m_bIsConnect = false;
    }

    void CDobotClient::DestroyConstruct()
    {
        m_bIsConnect = false;
        if (INVALID_SOCKET != m_sockListen)
        {
            shutdown(m_sockListen, SD_BOTH);
            closesocket(m_sockListen);
            m_sockListen = INVALID_SOCKET;
        }
    }

    std::string CDobotClient::GetIp() const
    {
        return m_strIp;
    }

    unsigned short CDobotClient::GetPort() const
    {
        return m_iPort;
    }

    bool CDobotClient::Connect(std::string strIp, unsigned short iPort)
    {
        if (INVALID_SOCKET == m_sockListen)
        {
            m_sockListen = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (INVALID_SOCKET == m_sockListen)
            {
                printf("create socket Error: (errcode: %d)\n", WSAGetLastError());
                return false;
            }
        }

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(iPort);
        inet_pton(addr.sin_family, strIp.c_str(), &(addr.sin_addr.s_addr));

        if (SOCKET_ERROR == ::connect(m_sockListen, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)))
        {
            printf("connect Error: (errcode: %d)\n", WSAGetLastError());
            return false;
        }

        m_strIp = strIp;
        m_iPort = iPort;
        m_bIsConnect = true;

        OnConnected();

        return true;
    }

    void CDobotClient::Disconnect()
    {
        m_bIsConnect = false;
        if (INVALID_SOCKET != m_sockListen)
        {
            shutdown(m_sockListen, SD_BOTH);
            closesocket(m_sockListen);
            m_sockListen = INVALID_SOCKET;
            OnDisconnected();
        }
    }

    bool CDobotClient::IsConnected() const
    {
        return m_bIsConnect;
    }

    bool CDobotClient::SendData(std::string str)
    {
        if (!IsConnected())
        {
            return false;
        }
        const char* pszData = str.c_str();
        size_t iHasWrite = 0;
        do
        {
            int iWrite = send(m_sockListen, pszData + iHasWrite,
                              static_cast<int>(str.length() - iHasWrite), 0);
            if (iWrite <= 0)
            {
                printf("send Error: (errcode: %d)\n", WSAGetLastError());
                Disconnect();
                return false;
            }
            iHasWrite += static_cast<size_t>(iWrite);
        } while (iHasWrite < str.length());
        return true;
    }

    std::string CDobotClient::WaitReply(int iTimeoutMillsecond)
    {
        fd_set fdRead;
        FD_ZERO(&fdRead);
        FD_SET(m_sockListen, &fdRead);
        struct timeval tv{};
        tv.tv_sec = iTimeoutMillsecond / 1000;
        tv.tv_usec = (iTimeoutMillsecond % 1000) * 1000;

        int iRet = select(static_cast<int>(fdRead.fd_count), &fdRead, nullptr, nullptr, &tv);
        if (iRet < 0)
        {
            printf("select Error: (errcode: %d)\n", WSAGetLastError());
            Disconnect();
            return "";
        }
        else if (0 == iRet)
        {
            return "";
        }

        char szBuf[1024] = "";
        int iLen = 1023;
        int iRead = recv(m_sockListen, szBuf, iLen, 0);
        if (iRead <= 0)
        {
            Disconnect();
            return "";
        }
        return std::string(szBuf);
    }

    int CDobotClient::Receive(char* pBuffer, int iLen)
    {
        int iRead = recv(m_sockListen, pBuffer, iLen, 0);
        if (iRead <= 0)
        {
            Disconnect();
        }
        return iRead;
    }
} // namespace Dobot
