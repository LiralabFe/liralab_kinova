#ifndef AUROVAS_SOCKET_HPP
#define AUROVAS_SOCKET_HPP

#include <iostream>
#include <string>
#include <RobotState.hpp>
#include <unistd.h>
#include <arpa/inet.h>
#include <kdl/kdl.hpp>
#include <vector>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>

class AurovasSocket
{
private:
    int sock;
    std::string target_ip;
    uint16_t target_port;

    std::vector<std::string> split(const std::string& s, char delim)
    {
        std::vector<std::string> result;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            result.push_back(item);
        }
        return result;
    }

public:
    AurovasSocket(std::string target_ip = "192.168.1.16", uint16_t target_port = 30004, std::string local_ip = "192.168.1.15", uint16_t port = 30004)
    {
        this->target_ip = target_ip;
        this->target_port = target_port;
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            perror("socket");
            return;
        }
        struct sockaddr_in local_addr;
        local_addr.sin_family = AF_INET;
        inet_pton(AF_INET, local_ip.c_str(), &local_addr.sin_addr);
        local_addr.sin_port = htons(port);
        if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            perror("bind");
            return;
        }
        std::cout << "Socket opened" << std::endl;
    }

    ~AurovasSocket()
    {
        CloseSocket();
    }

    void CloseSocket()
    {
        close(sock);
    }

    std::vector<std::string> Read()
    {
        char buffer[1024];
        struct sockaddr_in sender;
        socklen_t sender_len = sizeof(sender);
        int bytes = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&sender, &sender_len);
        if (bytes < 0) {
            perror("recvfrom");
            return {};
        }
        buffer[bytes] = '\0';
        std::string data(buffer);
        std::cout << "<<<--- " << data << std::endl;
        return split(data, ';');
    }

    void Write(const std::string& msg)
    {
        std::cout << "--->>> " << msg << std::endl;
        struct sockaddr_in target_addr;
        target_addr.sin_family = AF_INET;
        inet_pton(AF_INET, target_ip.c_str(), &target_addr.sin_addr);
        target_addr.sin_port = htons(target_port);
        sendto(sock, msg.c_str(), msg.size(), 0, (struct sockaddr*)&target_addr, sizeof(target_addr));
    }
};

#endif // AUROVAS_SOCKET_HPP