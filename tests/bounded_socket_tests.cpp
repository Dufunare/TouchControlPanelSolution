#include "../shared/BoundedSocketWrite.h"
#include <cassert>
#include <iostream>
#include <thread>

int main() {
    WSADATA data{};assert(WSAStartup(MAKEWORD(2,2),&data)==0);
    SOCKET listener=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    assert(bind(listener,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0);
    assert(listen(listener,1)==0);int size=sizeof(address);
    assert(getsockname(listener,reinterpret_cast<sockaddr*>(&address),&size)==0);
    SOCKET writer=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    assert(connect(writer,reinterpret_cast<sockaddr*>(&address),size)==0);
    SOCKET reader=accept(listener,nullptr,nullptr);assert(reader!=INVALID_SOCKET);
    u_long nonblocking=1;assert(ioctlsocket(writer,FIONBIO,&nonblocking)==0);
    const std::string payload(1024*1024,'x');std::string received;
    std::thread drain([&]{char buffer[4096];while(received.size()<payload.size()){
        const int n=recv(reader,buffer,sizeof(buffer),0);if(n<=0)break;received.append(buffer,n);
    }});
    assert(touchpanel::transport::boundedSocketWrite(writer,payload,2000));
    drain.join();assert(received==payload);
    int small=1024;setsockopt(writer,SOL_SOCKET,SO_SNDBUF,reinterpret_cast<char*>(&small),sizeof(small));
    setsockopt(reader,SOL_SOCKET,SO_RCVBUF,reinterpret_cast<char*>(&small),sizeof(small));
    const auto start=std::chrono::steady_clock::now();
    assert(!touchpanel::transport::boundedSocketWrite(writer,std::string(32*1024*1024,'y'),100));
    const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
    assert(elapsed<750); // scheduling allowance; never wait for peer to drain
    shutdown(writer,SD_BOTH);closesocket(writer);closesocket(reader);closesocket(listener);WSACleanup();
    std::cout<<"Partial writes preserve payload; stalled peer fails within bounded deadline ("<<elapsed<<" ms)\n";
}
