#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <mutex>
#include <optional>
#include <thread>

#include "gc_state.h"

class RoboCupGameControlData;

class GameController : public GCInterface {
public:
    GameController(uint8_t team_nr, uint8_t player_idx);
    ~GameController() override;

private:
    bool isNewPackage(const RoboCupGameControlData& gcData);
    void run();
    void send_heartbeats();
    void handle_socket_error(const char* op, const boost::system::error_code& ec);

    constexpr static int PACKAGES_PER_SECOND = 2;
    constexpr static int PACKAGES_UNTIL_RESET = PACKAGES_PER_SECOND * 3;

    int32_t numberOfOldPackagesReceived = 0;
    int32_t lastValidGameControllerTimestamp = INT32_MAX;
    uint8_t team_nr;
    uint8_t player_idx;
    GCState state;
    std::thread controller_thread;
    std::thread heartbeat_thread;
    std::optional<boost::asio::ip::udp::endpoint> heartbeat_endpoint;
    std::mutex heartbeat_mutex;
    std::atomic<bool> running{true};
};
