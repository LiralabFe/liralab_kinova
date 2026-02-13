#ifndef SOCKET_LIRALAB
#define SOCKET_LIRALAB

#include <iostream>
#include <string>
#include <RobotState.hpp>
#include <unistd.h>
#include <arpa/inet.h>

namespace KinovaLiralab
{
    class SocketLiralab
    {
    private:
        int _server;
        int _client;
        char buffer[1024] = {0};
    public:
        SocketLiralab(uint16_t);
        ~SocketLiralab();
        void CloseSocket();
        int WriteRobotState(const KinovaLiralab::RobotState&);
        int Write(const std::string&);
        auto Read() -> std::string;
    };
    
    SocketLiralab::SocketLiralab(uint16_t port)
    {
        _server = socket(AF_INET, SOCK_STREAM, 0);
        if (_server < 0) {perror("socket"); return;}

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if (bind(_server, (sockaddr*)&addr, sizeof(addr)) < 0) {perror("bind");return;}
        listen(_server, 1);
        std::cout << "Server in ascolto ...\n";
        _client = accept(_server, nullptr, nullptr);
        if (_client < 0) {perror("accept");return;}
    }

    int SocketLiralab::WriteRobotState(const KinovaLiralab::RobotState& robotState)
    {
        std::string msg{""};
        for(auto f : robotState._eePose)
        {   
            msg.append(std::to_string(f));
            msg.append(";");
        }
        std::cout << ">>> " << msg << "\n";
        return Write(msg);
    }

    int SocketLiralab::Write(const std::string& msg)
    {
        ssize_t ret = send(_client, (msg + "\n").c_str(), msg.size(), MSG_NOSIGNAL);
        if(ret < 0)
        {
            std::cout << "Errore nella send della socket." << std::endl;
            return -1;
        }
        return 0;
    }

    std::string SocketLiralab::Read()
    {
        ssize_t bytes = recv(_client, buffer, sizeof(buffer) - 1, 0);
        if (bytes <= 0) { perror("recv");return ""; }
        std::string received(buffer);
        std::cout << "<<< " << received << "\n";
        return received;
    }
    
    void SocketLiralab::CloseSocket()
    {
        close(_server);
        close(_client);
    }

    SocketLiralab::~SocketLiralab()
    {
        CloseSocket();
    }
    
}

#endif SOCKET_LIRALAB