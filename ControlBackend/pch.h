#ifndef PCH_H
#define PCH_H

// 1. 强制定义 WIN32 宏。很多老版本的 OpenHaptics 头文件强依赖这个宏来展开 HDAPIENTRY
#ifndef WIN32
#define WIN32
#endif

// 2. WinSock2 必须在 windows.h 之前引入 (Dobot TCP 通信模块需要)
#include <WinSock2.h>
#include <WS2tcpip.h>

// 3. 引入 Windows 核心头文件 (它必须在 HD/hd.h 之前)
#include <windows.h>

// 4. C++ 标准库
#include <atomic>
#include <cstdint>
#include <string>

// 5. 最后才是 OpenHaptics SDK
#include <HD/hd.h>

#endif //PCH_H