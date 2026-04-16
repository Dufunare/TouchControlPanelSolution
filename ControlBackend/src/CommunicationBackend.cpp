#include "pch.h"
#include "CommunicationBackend.h"

#include <atomic>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace touchpanel
{
    namespace
    {
        constexpr std::uint16_t kTransitDefaultPort = 8888;
        constexpr std::uint16_t kRobotControlPort = 29999;
        constexpr std::uint16_t kRobotMotionPort = 30003;

        constexpr double kMotionXMin = -350.0;
        constexpr double kMotionXMax = 650.0;
        constexpr double kMotionYMin = -350.0;
        constexpr double kMotionYMax = 350.0;
        constexpr double kMotionZMin = 50.0;
        constexpr double kMotionZMax = 500.0;

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

        std::string trimSpace(std::string text)
        {
            while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front())))
            {
                text.erase(text.begin());
            }

            while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())))
            {
                text.pop_back();
            }

            return text;
        }

        std::string extractCommandName(const std::string& commandText)
        {
            const std::string trimmed = trimSpace(commandText);
            const auto bracket = trimmed.find('(');
            if (bracket == std::string::npos)
            {
                return trimmed;
            }

            return trimSpace(trimmed.substr(0, bracket));
        }

        double clampValue(double value, double minValue, double maxValue)
        {
            return (value < minValue) ? minValue : ((value > maxValue) ? maxValue : value);
        }

        std::string formatDouble3(double value)
        {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3) << value;
            return oss.str();
        }

        struct ParsedReply
        {
            bool valid = false;
            int errorId = -1;
            std::vector<double> values;
            std::string commandName;
            std::string messageName;
        };

        ParsedReply parseTransitReply(const std::string& payload)
        {
            ParsedReply parsed;

            std::string text = trimSpace(payload);
            if (!text.empty() && text.back() == ';')
            {
                text.pop_back();
            }

            const auto firstComma = text.find(',');
            if (firstComma == std::string::npos)
            {
                return parsed;
            }

            try
            {
                parsed.errorId = std::stoi(text.substr(0, firstComma));
            }
            catch (...)
            {
                return parsed;
            }

            const auto valueOpen = text.find('{', firstComma + 1);
            if (valueOpen == std::string::npos)
            {
                return parsed;
            }

            const auto valueClose = text.find('}', valueOpen + 1);
            if (valueClose == std::string::npos)
            {
                return parsed;
            }

            const std::string valueBlock = text.substr(valueOpen + 1, valueClose - valueOpen - 1);
            if (!valueBlock.empty())
            {
                size_t start = 0;
                while (start < valueBlock.size())
                {
                    const auto comma = valueBlock.find(',', start);
                    const auto end = (comma == std::string::npos) ? valueBlock.size() : comma;
                    const std::string token = trimSpace(valueBlock.substr(start, end - start));
                    if (!token.empty())
                    {
                        try
                        {
                            parsed.values.push_back(std::stod(token));
                        }
                        catch (...)
                        {
                        }
                    }

                    if (comma == std::string::npos)
                    {
                        break;
                    }

                    start = comma + 1;
                }
            }

            const auto secondComma = text.find(',', valueClose + 1);
            if (secondComma == std::string::npos)
            {
                return parsed;
            }

            parsed.messageName = trimSpace(text.substr(secondComma + 1));
            const auto bracket = parsed.messageName.find('(');
            parsed.commandName = (bracket == std::string::npos)
                ? parsed.messageName
                : parsed.messageName.substr(0, bracket);

            while (!parsed.commandName.empty() && std::isspace(static_cast<unsigned char>(parsed.commandName.back())))
            {
                parsed.commandName.pop_back();
            }

            parsed.valid = !parsed.commandName.empty();
            return parsed;
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

        std::string defaultIp = "192.168.100.194";
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

        std::mutex robotStateMutex;
        int lastRobotMode = -1;
        bool hasLastPose = false;
        std::array<double, 6> lastPose{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

        std::mutex motionLogMutex;
        std::chrono::steady_clock::time_point lastMotionLogTime{};
        int motionSentSinceLastLog = 0;

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

        bool sendLine(const std::string& line, bool reportTx)
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

            int totalSent = 0;
            const int totalSize = static_cast<int>(payload.size());
            while (totalSent < totalSize)
            {
                const int sent = ::send(
                    socketHandle,
                    payload.data() + totalSent,
                    totalSize - totalSent,
                    0);
                if (sent == SOCKET_ERROR)
                {
                    emitMessage("发送失败：" + std::to_string(WSAGetLastError()));
                    return false;
                }

                if (sent == 0)
                {
                    emitMessage("发送失败：连接已关闭。");
                    return false;
                }

                totalSent += sent;
            }

            if (reportTx)
            {
                emitMessage("[TCP-TX] " + line);
                emitTx(line);
            }
            return true;
        }

        void sendMotionCommand(double x, double y, double z)
        {
            if (!connected.load(std::memory_order_relaxed))
            {
                return;
            }

            const double mappedX = clampValue(x, kMotionXMin, kMotionXMax);
            const double mappedY = clampValue(y, kMotionYMin, kMotionYMax);
            const double mappedZ = clampValue(z, kMotionZMin, kMotionZMax);

            const std::string command =
                "ServoP(" + formatDouble3(mappedX) + "," + formatDouble3(mappedY) + "," + formatDouble3(mappedZ) + ",150,0,90)";
            const std::string packet = std::to_string(kRobotMotionPort) + "|" + command;

            if (!sendLine(packet, false))
            {
                return;
            }

            const auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(motionLogMutex);
            ++motionSentSinceLastLog;
            if (lastMotionLogTime.time_since_epoch().count() == 0 ||
                now - lastMotionLogTime >= std::chrono::seconds(1))
            {
                emitMessage("[MOTION] ServoP 流发送中，本秒已发送 " + std::to_string(motionSentSinceLastLog) + " 条。");
                motionSentSinceLastLog = 0;
                lastMotionLogTime = now;
            }
        }

        void updateRobotSnapshot(const ParsedReply& parsed)
        {
            if (parsed.errorId != 0)
            {
                return;
            }

            std::lock_guard<std::mutex> lock(robotStateMutex);
            if (parsed.commandName == "RobotMode" && !parsed.values.empty())
            {
                lastRobotMode = static_cast<int>(std::llround(parsed.values[0]));
            }
            else if (parsed.commandName == "GetPose" && parsed.values.size() >= 6)
            {
                for (size_t i = 0; i < 6; ++i)
                {
                    lastPose[i] = parsed.values[i];
                }
                hasLastPose = true;
            }
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

        void finishPendingByReplyObserved(bool success, const std::string& detail)
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

            emitOperation(operation, success, detail);
            if (success)
            {
                emitMessage("[OP] " + operation + " 执行成功：" + detail);
            }
            else
            {
                emitMessage("[OP] " + operation + " 执行失败：" + detail);
            }
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
            const auto parsed = parseTransitReply(detail.empty() ? rawLine : detail);
            if (parsed.valid)
            {
                updateRobotSnapshot(parsed);
            }

            bool hasPending = false;
            std::string pendingOperation;
            std::string pendingCommand;
            {
                std::lock_guard<std::mutex> lock(pendingMutex);
                hasPending = pending.active;
                if (hasPending)
                {
                    pendingOperation = pending.operation;
                    pendingCommand = pending.command;
                }
            }

            if (hasPending)
            {
                const std::string expectedCommandName = extractCommandName(pendingCommand);

                if (!parsed.valid)
                {
                    emitMessage("[OP] " + pendingOperation + " 收到未识别回包，继续等待：" + (detail.empty() ? rawLine : detail));
                    return;
                }

                if (!expectedCommandName.empty() && parsed.commandName != expectedCommandName)
                {
                    emitMessage("[OP] " + pendingOperation + " 收到异步回包(" + parsed.commandName + ")，期望 " + expectedCommandName + "，继续等待。");
                    return;
                }

                const bool success = (parsed.errorId == 0);
                if (success)
                {
                    finishPendingByReplyObserved(true, parsed.messageName);
                    emitRobotStatus("命令成功：" + parsed.messageName);
                }
                else
                {
                    finishPendingByReplyObserved(false, "ErrorID=" + std::to_string(parsed.errorId) + " " + parsed.messageName);
                    emitRobotStatus("命令失败：ErrorID=" + std::to_string(parsed.errorId));
                }
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
                SOCKET currentSocket = INVALID_SOCKET;
                int received = 0;
                {
                    std::lock_guard<std::mutex> lock(socketMutex);
                    if (socketHandle == INVALID_SOCKET)
                    {
                        break;
                    }

                    currentSocket = socketHandle;
                }

                received = ::recv(currentSocket, recvBuffer.data(), static_cast<int>(recvBuffer.size()), 0);

                if (received <= 0)
                {
                    break;
                }

                buffer.append(recvBuffer.data(), static_cast<size_t>(received));

                while (true)
                {
                    const auto lineEnd = buffer.find_first_of("\r\n\0");
                    if (lineEnd == std::string::npos)
                    {
                        break;
                    }

                    std::string line = buffer.substr(0, lineEnd);

                    size_t eraseCount = lineEnd + 1;
                    while (eraseCount < buffer.size() && (buffer[eraseCount] == '\r' || buffer[eraseCount] == '\n' || buffer[eraseCount] == '\0'))
                    {
                        ++eraseCount;
                    }

                    buffer.erase(0, eraseCount);

                    line = trimCr(line);
                    if (!line.empty())
                    {
                        onLineReceived(line);
                    }
                }

                bool hasPending = false;
                {
                    std::lock_guard<std::mutex> lock(pendingMutex);
                    hasPending = pending.active;
                }

                if (hasPending && !buffer.empty())
                {
                    std::string line = trimCr(buffer);
                    buffer.clear();
                    if (!line.empty())
                    {
                        onLineReceived(line);
                    }
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
            if (!sendLine(packet, true))
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

        disconnectTransitInternal(true);

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
        disconnectTransitInternal(false);
    }

    void CommunicationBackend::disconnectTransitInternal(bool silentIfNotConnected)
    {
        const bool wasConnected = m_impl->connected.load(std::memory_order_relaxed);
        if (!wasConnected)
        {
            if (!silentIfNotConnected)
            {
                m_impl->emitMessage("当前并未连接中转站。");
            }
            m_impl->finishPendingAsCanceled("连接未建立");
            return;
        }

        m_impl->connected.store(false, std::memory_order_relaxed);

        m_impl->stopThreads();

        m_impl->emitConnection(false);
        m_impl->emitMessage("中转站连接已断开。");
        m_impl->finishPendingAsCanceled("连接断开，命令未完成。");
    }

    bool CommunicationBackend::isTransitConnected() const
    {
        return m_impl->connected.load(std::memory_order_relaxed);
    }

    void CommunicationBackend::sendMotion(double x, double y, double z)
    {
        m_impl->sendMotionCommand(x, y, z);
    }

    void CommunicationBackend::requestRobotMode()
    {
        m_impl->sendRobotCommand("RobotMode", "RobotMode()", 3000);
    }

    void CommunicationBackend::requestCurrentPose()
    {
        m_impl->sendRobotCommand("GetPose", "GetPose()", 3000);
    }

    bool CommunicationBackend::isRobotModeReady() const
    {
        std::lock_guard<std::mutex> lock(m_impl->robotStateMutex);
        return m_impl->lastRobotMode == 5;
    }

    bool CommunicationBackend::tryGetLastPose(std::array<double, 6>& pose) const
    {
        std::lock_guard<std::mutex> lock(m_impl->robotStateMutex);
        if (!m_impl->hasLastPose)
        {
            return false;
        }

        pose = m_impl->lastPose;
        return true;
    }

    void CommunicationBackend::powerOn()
    {
        m_impl->sendRobotCommand("PowerOn", "PowerOn()", 15000);
    }

    void CommunicationBackend::enableRobot()
    {
        m_impl->sendRobotCommand("EnableRobot", "EnableRobot()", 5000);
    }

    void CommunicationBackend::disableRobot()
    {
        m_impl->sendRobotCommand("DisableRobot", "DisableRobot()", 5000);
    }

    void CommunicationBackend::clearError()
    {
        m_impl->sendRobotCommand("ClearError", "ClearError()", 5000);
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
