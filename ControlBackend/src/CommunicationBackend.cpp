#include "pch.h"
#include "CommunicationBackend.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace touchpanel
{
    namespace
    {
        constexpr std::uint16_t kTransitDefaultPort = 9000;
        constexpr std::uint16_t kRobotControlPort = 29999;

        std::string trimCr(std::string line)
        {
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }
            return line;
        }

        std::string extractPayload(const std::string& raw)
        {
            const auto separator = raw.find('|');
            if (separator == std::string::npos || separator == 0)
            {
                return raw;
            }

            return raw.substr(separator + 1);
        }
    }

    struct CommunicationBackend::Impl
    {
        MessageCallback messageCallback;
        ConnectionCallback connectionCallback;
        TextCallback robotStatusCallback;
        TextCallback txCallback;
        TextCallback rxCallback;
        OperationCallback operationCallback;

        std::mutex callbackMutex;

        SOCKET socketHandle = INVALID_SOCKET;
        std::mutex socketMutex;

        std::atomic<bool> connected{ false };
        std::atomic<bool> stopRequested{ false };

        std::thread recvThread;
        std::thread timeoutThread;

        std::string defaultIp = "127.0.0.1";
        std::uint16_t defaultPort = kTransitDefaultPort;

        struct PendingCommand
        {
            bool active = false;
            std::string operation;
            std::string command;
            std::chrono::steady_clock::time_point deadline{};
        };

        PendingCommand pending;
        std::mutex pendingMutex;

        void emitMessage(const std::string& text)
        {
            MessageCallback callback;
            {
                std::lock_guard<std::mutex> lock(callbackMutex);
                callback = messageCallback;
            }

            if (callback)
            {
                callback(text);
            }
        }

        void emitConnection(bool isConnected)
        {
            ConnectionCallback callback;
            {
                std::lock_guard<std::mutex> lock(callbackMutex);
                callback = connectionCallback;
            }

            if (callback)
            {
                callback(isConnected);
            }
        }

        void emitRobotStatus(const std::string& text)
        {
            TextCallback callback;
            {
                std::lock_guard<std::mutex> lock(callbackMutex);
                callback = robotStatusCallback;
            }

            if (callback)
            {
                callback(text);
            }
        }

        void emitTx(const std::string& text)
        {
            TextCallback callback;
            {
                std::lock_guard<std::mutex> lock(callbackMutex);
                callback = txCallback;
            }

            if (callback)
            {
                callback(text);
            }
        }

        void emitRx(const std::string& text)
        {
            TextCallback callback;
            {
                std::lock_guard<std::mutex> lock(callbackMutex);
                callback = rxCallback;
            }

            if (callback)
            {
                callback(text);
            }
        }

        void emitOperation(const std::string& operation, bool success, const std::string& detail)
        {
            OperationCallback callback;
            {
                std::lock_guard<std::mutex> lock(callbackMutex);
                callback = operationCallback;
            }

            if (callback)
            {
                callback(operation, success, detail);
            }
        }

        bool sendLine(const std::string& line)
        {
            if (!connected.load(std::memory_order_relaxed))
            {
                emitMessage("发送失败：TCP 未连接。");
                return false;
            }

            std::string payload = line;
            if (payload.empty() || payload.back() != '\n')
            {
                payload.push_back('\n');
            }

            std::lock_guard<std::mutex> lock(socketMutex);
            if (socketHandle == INVALID_SOCKET)
            {
                emitMessage("发送失败：socket 无效。");
                return false;
            }

            const int sent = ::send(socketHandle, payload.data(), static_cast<int>(payload.size()), 0);
            if (sent == SOCKET_ERROR)
            {
                emitMessage("发送失败：" + std::to_string(WSAGetLastError()));
                return false;
            }

            emitMessage("[TCP-TX] " + line);
            emitTx(line);
            return true;
        }

        void finishPendingTimeout()
        {
            std::string operation;
            {
                std::lock_guard<std::mutex> lock(pendingMutex);
                if (!pending.active)
                {
                    return;
                }

                operation = pending.operation;
                pending = {};
            }

            emitOperation(operation, false, "timeout");
            emitMessage("[OP] " + operation + " 超时：等待机械臂回复超时。");
        }

        void finishPendingByReplyObserved(const std::string& detail)
        {
            std::string operation;
            {
                std::lock_guard<std::mutex> lock(pendingMutex);
                if (!pending.active)
                {
                    return;
                }

                operation = pending.operation;
                pending = {};
            }

            emitOperation(operation, false, "got_reply");
            emitMessage("[OP] " + operation + " 已收到回包，待规则确认：" + detail);
        }

        void finishPendingAsCanceled(const std::string& reason)
        {
            std::string operation;
            {
                std::lock_guard<std::mutex> lock(pendingMutex);
                if (!pending.active)
                {
                    return;
                }

                operation = pending.operation;
                pending = {};
            }

            emitOperation(operation, false, reason);
            emitMessage("[OP] " + operation + " 已取消：" + reason);
        }

        void onLineReceived(const std::string& rawLine)
        {
            emitMessage("[TCP-RX] " + rawLine);
            emitRx(rawLine);

            const std::string detail = extractPayload(rawLine);

            bool hasPending = false;
            {
                std::lock_guard<std::mutex> lock(pendingMutex);
                hasPending = pending.active;
            }

            if (hasPending)
            {
                finishPendingByReplyObserved(detail.empty() ? rawLine : detail);
                return;
            }

            emitMessage("收到中转站消息：" + (detail.empty() ? rawLine : detail));
        }

        void recvLoop()
        {
            std::string buffer;
            std::vector<char> recvBuffer(4096);

            while (!stopRequested.load(std::memory_order_relaxed))
            {
                int received = 0;
                {
                    std::lock_guard<std::mutex> lock(socketMutex);
                    if (socketHandle == INVALID_SOCKET)
                    {
                        break;
                    }

                    received = ::recv(socketHandle, recvBuffer.data(), static_cast<int>(recvBuffer.size()), 0);
                }

                if (received <= 0)
                {
                    break;
                }

                buffer.append(recvBuffer.data(), static_cast<size_t>(received));

                while (true)
                {
                    const auto lineEnd = buffer.find('\n');
                    if (lineEnd == std::string::npos)
                    {
                        break;
                    }

                    std::string line = trimCr(buffer.substr(0, lineEnd));
                    buffer.erase(0, lineEnd + 1);
                    onLineReceived(line);
                }
            }

            if (!stopRequested.load(std::memory_order_relaxed))
            {
                connected.store(false, std::memory_order_relaxed);
                emitConnection(false);
                emitMessage("中转站连接已断开。");
                finishPendingAsCanceled("连接断开，命令未完成。");
            }
        }

        void timeoutLoop()
        {
            while (!stopRequested.load(std::memory_order_relaxed))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                bool timeout = false;
                {
                    std::lock_guard<std::mutex> lock(pendingMutex);
                    if (pending.active && std::chrono::steady_clock::now() > pending.deadline)
                    {
                        timeout = true;
                    }
                }

                if (timeout)
                {
                    finishPendingTimeout();
                }
            }
        }

        void stopThreads()
        {
            stopRequested.store(true, std::memory_order_relaxed);

            {
                std::lock_guard<std::mutex> lock(socketMutex);
                if (socketHandle != INVALID_SOCKET)
                {
                    ::shutdown(socketHandle, SD_BOTH);
                    ::closesocket(socketHandle);
                    socketHandle = INVALID_SOCKET;
                }
            }

            if (recvThread.joinable())
            {
                recvThread.join();
            }

            if (timeoutThread.joinable())
            {
                timeoutThread.join();
            }
        }

        void sendRobotCommand(const std::string& operation, const std::string& command, int timeoutMs)
        {
            if (!connected.load(std::memory_order_relaxed))
            {
                emitMessage("未连接中转站，无法执行 " + operation + "。");
                emitOperation(operation, false, "TCP 未连接");
                return;
            }

            {
                std::lock_guard<std::mutex> lock(pendingMutex);
                if (pending.active)
                {
                    emitMessage("当前仍有命令在执行，请稍后再试。");
                    emitOperation(operation, false, "上一条命令尚未完成");
                    return;
                }

                pending.active = true;
                pending.operation = operation;
                pending.command = command;
                pending.deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
            }

            const std::string packet = std::to_string(kRobotControlPort) + "|" + command;
            if (!sendLine(packet))
            {
                finishPendingAsCanceled("发送失败");
                return;
            }

            emitMessage("已发送 " + command + "，等待机械臂回复...");
        }
    };

    CommunicationBackend::CommunicationBackend()
        : m_impl(new Impl())
    {
        WSADATA wsaData{};
        WSAStartup(MAKEWORD(2, 2), &wsaData);
        m_impl->emitRobotStatus("未知（等待机械臂上报）");
    }

    CommunicationBackend::~CommunicationBackend()
    {
        if (m_impl == nullptr)
        {
            return;
        }

        disconnectTransit();
        WSACleanup();
        delete m_impl;
        m_impl = nullptr;
    }

    void CommunicationBackend::setMessageCallback(MessageCallback callback)
    {
        std::lock_guard<std::mutex> lock(m_impl->callbackMutex);
        m_impl->messageCallback = std::move(callback);
    }

    void CommunicationBackend::setConnectionCallback(ConnectionCallback callback)
    {
        std::lock_guard<std::mutex> lock(m_impl->callbackMutex);
        m_impl->connectionCallback = std::move(callback);
    }

    void CommunicationBackend::setRobotStatusCallback(TextCallback callback)
    {
        std::lock_guard<std::mutex> lock(m_impl->callbackMutex);
        m_impl->robotStatusCallback = std::move(callback);
    }

    void CommunicationBackend::setTcpTxCallback(TextCallback callback)
    {
        std::lock_guard<std::mutex> lock(m_impl->callbackMutex);
        m_impl->txCallback = std::move(callback);
    }

    void CommunicationBackend::setTcpRxCallback(TextCallback callback)
    {
        std::lock_guard<std::mutex> lock(m_impl->callbackMutex);
        m_impl->rxCallback = std::move(callback);
    }

    void CommunicationBackend::setOperationCallback(OperationCallback callback)
    {
        std::lock_guard<std::mutex> lock(m_impl->callbackMutex);
        m_impl->operationCallback = std::move(callback);
    }

    void CommunicationBackend::connectTransit(const std::string& ipOverride, std::uint16_t portOverride)
    {
        const std::string host = ipOverride.empty() ? m_impl->defaultIp : ipOverride;
        const std::uint16_t port = (portOverride == 0) ? m_impl->defaultPort : portOverride;

        disconnectTransit();

        m_impl->emitMessage("正在连接中转站 " + host + ":" + std::to_string(port) + " ...");

        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;

        addrinfo* result = nullptr;
        const int resolveRet = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &result);
        if (resolveRet != 0 || result == nullptr)
        {
            m_impl->emitMessage("地址解析失败。");
            return;
        }

        SOCKET newSocket = ::socket(result->ai_family, result->ai_socktype, result->ai_protocol);
        if (newSocket == INVALID_SOCKET)
        {
            freeaddrinfo(result);
            m_impl->emitMessage("创建 socket 失败。");
            return;
        }

        const int connectRet = ::connect(newSocket, result->ai_addr, static_cast<int>(result->ai_addrlen));
        freeaddrinfo(result);

        if (connectRet == SOCKET_ERROR)
        {
            ::closesocket(newSocket);
            m_impl->emitMessage("连接中转站失败：" + std::to_string(WSAGetLastError()));
            return;
        }

        {
            std::lock_guard<std::mutex> lock(m_impl->socketMutex);
            m_impl->socketHandle = newSocket;
        }

        m_impl->stopRequested.store(false, std::memory_order_relaxed);
        m_impl->connected.store(true, std::memory_order_relaxed);

        m_impl->recvThread = std::thread([this]() { m_impl->recvLoop(); });
        m_impl->timeoutThread = std::thread([this]() { m_impl->timeoutLoop(); });

        m_impl->emitConnection(true);
        m_impl->emitMessage("中转站 TCP 已建立连接。");
    }

    void CommunicationBackend::disconnectTransit()
    {
        const bool wasConnected = m_impl->connected.load(std::memory_order_relaxed);
        if (!wasConnected)
        {
            m_impl->emitMessage("当前并未连接中转站。");
            m_impl->finishPendingAsCanceled("连接未建立");
            return;
        }

        m_impl->connected.store(false, std::memory_order_relaxed);

        m_impl->stopThreads();

        m_impl->emitConnection(false);
        m_impl->emitMessage("中转站连接已断开。");
        m_impl->finishPendingAsCanceled("连接断开，命令未完成。");
    }

    void CommunicationBackend::powerOn()
    {
        m_impl->sendRobotCommand("PowerOn", "PowerOn()", 15000);
    }

    void CommunicationBackend::enableRobot()
    {
        m_impl->sendRobotCommand("EnableRobot", "EnableRobot()", 5000);
    }

    void CommunicationBackend::emergencyStop()
    {
        m_impl->sendRobotCommand("EmergencyStop", "EmergencyStop()", 5000);
    }

    void CommunicationBackend::startDrag()
    {
        m_impl->sendRobotCommand("StartDrag", "StartDrag()", 5000);
    }

    void CommunicationBackend::stopDrag()
    {
        m_impl->sendRobotCommand("StopDrag", "StopDrag()", 5000);
    }
}
