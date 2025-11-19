#include "tc_network.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <cerrno>
#include <cstring>
#include <loguru.hpp>
#include <sstream>

#include "named_thread.h"

void TeamComNetwork::sendBroadcast(uint16_t port, const std::string& message) {
    int sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        LOG_F(ERROR, "Failed to create send socket: %s", std::strerror(errno));
        return;
    }

    int broadcast_enable = 1;
    if (::setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast_enable,
                     sizeof(broadcast_enable)) < 0) {
        LOG_F(ERROR, "Failed to set broadcast option: %s", std::strerror(errno));
        ::close(sockfd);
        return;
    }

    struct sockaddr_in broadcast_addr;
    std::memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(port);
    broadcast_addr.sin_addr.s_addr = inet_addr("192.168.255.255");

    if (::sendto(sockfd, message.c_str(), message.length(), 0, (struct sockaddr*)&broadcast_addr,
                 sizeof(broadcast_addr)) < 0) {
        LOG_F(ERROR, "Failed to send broadcast message: %s", std::strerror(errno));
    }

    ::close(sockfd);
}

TeamComNetwork::TeamComNetwork(uint8_t team_id, bool sending_allowed) : team_id(team_id) {
    if (sending_allowed)
        sender = named_thread("tc_network_send", [this]() { send(); });
    receiver = named_thread("tc_network_recv", [this]() { receive(); });
}

void TeamComNetwork::receive() {
    int sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        LOG_F(ERROR, "Failed to create receive socket: %s", std::strerror(errno));
        return;
    }

    int reuse_addr = 1;
    if (::setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr)) < 0) {
        LOG_F(ERROR, "Failed to set reuse_address option: %s", std::strerror(errno));
        ::close(sockfd);
        return;
    }

    int broadcast_enable = 1;
    if (::setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast_enable,
                     sizeof(broadcast_enable)) < 0) {
        LOG_F(ERROR, "Failed to set broadcast option for receive: %s", std::strerror(errno));
        ::close(sockfd);
        return;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    if (::setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        LOG_F(WARNING, "Failed to set receive timeout: %s", std::strerror(errno));
    }

    int rcvbuf_size = 2048;
    if (::setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
        LOG_F(WARNING, "Failed to set SO_RCVBUF: %s", std::strerror(errno));
    }

    struct sockaddr_in listen_addr;
    std::memset(&listen_addr, 0, sizeof(listen_addr));
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(11000 + team_id);
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(sockfd, (struct sockaddr*)&listen_addr, sizeof(listen_addr)) < 0) {
        LOG_F(ERROR, "Failed to bind receive socket: %s", std::strerror(errno));
        ::close(sockfd);
        return;
    }

    const size_t max_buffer_size = 1024;
    char buffer[max_buffer_size];

    while (!shutdown) {
        struct sockaddr_in sender_addr;
        socklen_t sender_addr_len = sizeof(sender_addr);
        ssize_t bytes_received = ::recvfrom(sockfd, buffer, max_buffer_size, 0,
                                            (struct sockaddr*)&sender_addr, &sender_addr_len);

        if (bytes_received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // This is an expected timeout
                continue;
            }
            LOG_F(ERROR, "Error receiving data: %s", std::strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Filter to only accept addresses from 10.0.{team_id}.* subnet
        uint32_t sender_ip = ntohl(sender_addr.sin_addr.s_addr);
        uint8_t first_octet = (sender_ip >> 24) & 0xff;
        uint8_t second_octet = (sender_ip >> 16) & 0xff;
        uint8_t third_octet = (sender_ip >> 8) & 0xff;

        if (first_octet != 192 || second_octet != 168 || third_octet != team_id) {
            continue;
        }

        std::string message(buffer, bytes_received);
        std::stringstream ss(message);

        TeamComData data;
        try {
            boost::archive::binary_iarchive ia(ss);
            ia >> data;
            tc_internal::receive(data);
        } catch (const std::exception& e) {
            LOG_F(ERROR, "Failed to deserialize TeamComData: %s", e.what());
        }
    }

    ::close(sockfd);
}
