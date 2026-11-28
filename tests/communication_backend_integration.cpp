#include <winsock2.h>
#include "../ControlBackend/include/CommunicationBackend.h"
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

int main() {
    WSADATA data{};assert(WSAStartup(MAKEWORD(2,2),&data)==0);
    SOCKET listener=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    assert(bind(listener,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0);
    assert(listen(listener,2)==0);int size=sizeof(address);
    assert(getsockname(listener,reinterpret_cast<sockaddr*>(&address),&size)==0);
    touchpanel::CommunicationBackend backend;
    std::atomic<int> lost{0};backend.setConnectionCallback([&](bool connected){if(!connected)++lost;});
    for(int iteration=0;iteration<2;++iteration) {
        backend.connectTransit("127.0.0.1",ntohs(address.sin_port));
        assert(backend.isTransitConnected());
        SOCKET peer=accept(listener,nullptr,nullptr);assert(peer!=INVALID_SOCKET);
        DWORD timeout=2000;setsockopt(peer,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<char*>(&timeout),sizeof(timeout));
        std::string received;char buffer[512];
        while(received.find("RelayHeartbeat()") == std::string::npos) {
            int n=recv(peer,buffer,sizeof(buffer),0);assert(n>0);received.append(buffer,n);
        }
        // No motion is generated while the periodic lease remains alive.
        received.clear();int n=recv(peer,buffer,sizeof(buffer),0);assert(n>0);received.append(buffer,n);
        assert(received.find("RelayHeartbeat()")!=std::string::npos);
        assert(received.find("Servo")==std::string::npos);
        const char failure[]="ERROR|motion_interlocked|test;";
        assert(send(peer,failure,sizeof(failure)-1,0)>0);
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
        while(lost.load()<=iteration && std::chrono::steady_clock::now()<deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        assert(!backend.isTransitConnected() && lost.load()>iteration);
        // Reconnect must join old threads even though connected is already false.
        closesocket(peer);
    }
    backend.disconnectTransit();closesocket(listener);WSACleanup();
    std::cout<<"Real backend periodic heartbeat, relay rejection, disconnect notification and reconnect passed\n";
}
