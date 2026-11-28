#pragma once
#include <winsock2.h>
#include <chrono>
#include <string>
#include <algorithm>

namespace touchpanel::transport {
// Socket MUST be nonblocking. One deadline covers ALL partial writes, not
// a fresh timeout per fragment. On false the caller must retire the stream.
inline bool boundedSocketWrite(SOCKET socket, const std::string& bytes, int timeoutMs=100) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeoutMs);
    size_t offset=0;
    while(offset<bytes.size()) {
        const auto remaining=std::chrono::duration_cast<std::chrono::microseconds>(
            deadline-std::chrono::steady_clock::now()).count();
        if(remaining<=0)return false;
        fd_set writable;FD_ZERO(&writable);FD_SET(socket,&writable);
        timeval timeout{static_cast<long>(remaining/1000000),static_cast<long>(remaining%1000000)};
        if(::select(0,nullptr,&writable,nullptr,&timeout)<=0)return false;
        const int sent=::send(socket,bytes.data()+offset,
            static_cast<int>((std::min)(bytes.size()-offset,size_t(65536))),0);
        if(sent>0)offset+=sent;
        else if(sent==SOCKET_ERROR && WSAGetLastError()==WSAEWOULDBLOCK)continue;
        else return false;
    }
    return true;
}
}
