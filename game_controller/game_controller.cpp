#include "game_controller.h"

#include <boost/asio.hpp>
#include <chrono>
#include <cstdlib>
#include <loguru.hpp>
#include <thread>

#include "RoboCupGameControlData.h"
#include "gc_pub_sub.h"
#include "named_thread.h"

GameController::GameController(uint8_t team_nr, uint8_t player_idx)
    : team_nr(team_nr), player_idx(player_idx) {
    controller_thread = named_thread("game_controller", &GameController::run, this);
    heartbeat_thread = named_thread("gc_heartbeat", &GameController::send_heartbeats, this);
}

GameController::~GameController() {
    running = false;

    if (controller_thread.joinable()) {
        controller_thread.join();
    }

    if (heartbeat_thread.joinable()) {
        heartbeat_thread.join();
    }
}

void GameController::handle_socket_error(const char* op, const boost::system::error_code& ec) {
    if (ec) {
        LOG_F(FATAL, "%s socket failed: %s.", op, ec.message().c_str());
    }
}

void GameController::run() {
    boost::asio::io_context io_context;
    boost::system::error_code ec;
    boost::asio::ip::udp::socket socket(io_context);

    handle_socket_error("open", socket.open(boost::asio::ip::udp::v4(), ec));
    handle_socket_error("set_option",
                        socket.set_option(boost::asio::socket_base::reuse_address(true), ec));
    handle_socket_error("bind",
                        socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                                                   GAMECONTROLLER_DATA_PORT),
                                    ec));

    // Set socket to non-blocking mode to allow checking shutdown flag
    socket.non_blocking(true, ec);
    if (ec) {
        LOG_F(ERROR, "Failed to set socket non-blocking: %s", ec.message().c_str());
    }

    RoboCupGameControlData gcData{};
    while (running) {
        boost::asio::ip::udp::endpoint remoteEndpoint;
        size_t len = socket.receive_from(boost::asio::buffer(&gcData, sizeof(gcData)),
                                         remoteEndpoint, 0, ec);

        if (ec) {
            if (ec == boost::asio::error::would_block) {
                // No data available, sleep briefly and continue
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            LOG_F(ERROR, "receive_from failed: %s", ec.message().c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
        }

        if (std::strncmp(gcData.header, GAMECONTROLLER_STRUCT_HEADER,
                         strlen(GAMECONTROLLER_STRUCT_HEADER)) != 0) {
            std::string r_header(gcData.header, sizeof(gcData.header));
            LOG_F(ERROR, "Received GameController packet with wrong header: got %s, expected %s",
                  r_header.c_str(), GAMECONTROLLER_STRUCT_HEADER);
            continue;
        }

        if (gcData.version != GAMECONTROLLER_STRUCT_VERSION) {
            LOG_F(ERROR, "Received GameController packet with wrong version: got %d, expected %d",
                  gcData.version, GAMECONTROLLER_STRUCT_VERSION);
            continue;
        }

        if (len < sizeof(RoboCupGameControlData)) {
            LOG_F(ERROR,
                  "Received GameController packet with wrong length: got %zu, expected at least "
                  "%zu",
                  len, sizeof(RoboCupGameControlData));
            continue;
        }

        if (gcData.teams[0].teamNumber != team_nr && gcData.teams[1].teamNumber != team_nr)
            continue;

        if (!isNewPackage(gcData))
            continue;

        GCState new_state;
        new_state.player_idx = player_idx;

        int my_team_idx = gcData.teams[0].teamNumber == team_nr ? 0 : 1;
        // Not our game.
        if (gcData.teams[my_team_idx].teamNumber != team_nr)
            continue;
        new_state.my_team.team_nr = team_nr;
        // TODO: Implement substitudes better.
        for (int i = 0; i < gcData.playersPerTeam + 2; i++) {
            Player player;
            player.is_penalized = gcData.teams[my_team_idx].players[i].penalty != PENALTY_NONE;
            new_state.my_team.players.push_back(player);
        }
        new_state.opp_team.team_nr = gcData.teams[1 - my_team_idx].teamNumber;

        for (int i = 0; i < gcData.playersPerTeam + 2; i++) {
            Player player;
            player.is_penalized = gcData.teams[1 - my_team_idx].players[i].penalty != PENALTY_NONE;
            new_state.opp_team.players.push_back(player);
        }

        SecondaryState secondary_state = static_cast<SecondaryState>(gcData.secondaryState);
        if (secondary_state == SecondaryState::PenaltyShoot) {
            new_state.game_phase = GCState::GamePhase::PENALTY_SHOOT;
        } else if (gcData.firstHalf == 1) {
            new_state.game_phase = GCState::GamePhase::FIRST_HALF;
        } else {
            new_state.game_phase = GCState::GamePhase::SECOND_HALF;
        }
        new_state.state = static_cast<GameState>(gcData.state);
        new_state.secondary_state = secondary_state;
        if (gcData.kickOffTeam == DROPBALL)
            new_state.kicking_team = KickingTeam::Both;
        else
            new_state.kicking_team =
                    gcData.kickOffTeam == team_nr ? KickingTeam::MyTeam : KickingTeam::OppTeam;
        new_state.setPlay_kicking_team = state.setPlay_kicking_team;
        if (gcData.secondaryState != STATE2_NORMAL && gcData.secondaryState != STATE2_OVERTIME &&
            gcData.secondaryState != STATE2_TIMEOUT &&
            gcData.secondaryState != STATE2_PENALTYSHOOT) {
            new_state.setPlay_kicking_team = gcData.secondaryStateInfo[0] == team_nr
                                                     ? KickingTeam::MyTeam
                                                     : KickingTeam::OppTeam;
            new_state.setPlay_state = static_cast<GameState>(gcData.secondaryStateInfo[1]);
        }
        new_state.secs_remaining = gcData.secsRemaining;
        new_state.secondary_time = gcData.secondaryTime;
        // Humanoid doesn't have a message budget, so allow the max possible.
        new_state.remaining_message_budget = std::numeric_limits<uint16_t>::max();

        {
            std::lock_guard<std::mutex> lock(heartbeat_mutex);
            boost::asio::ip::udp::endpoint endpoint = remoteEndpoint;
            endpoint.port(GAMECONTROLLER_RETURN_PORT);
            heartbeat_endpoint = endpoint;
        }
        LOG_S(ERROR) << "GameState received: " << (int)new_state.state
                     << " Secondary: " << (int)new_state.secondary_state
                     << " SetPlay: " << (int)new_state.setPlay_state;
        gc_state.publish(new_state);

        if (state != new_state) {
            on_gc_state_change(state, new_state);
            state = new_state;
        }
    }
}

void GameController::send_heartbeats() {
    boost::asio::io_context io_context;
    boost::system::error_code ec;
    boost::asio::ip::udp::socket socket(io_context);

    handle_socket_error("open", socket.open(boost::asio::ip::udp::v4(), ec));

    RoboCupGameControlReturnData returnData;
    returnData.team = team_nr;
    returnData.player = player_idx + 1;
    returnData.message = GAMECONTROLLER_RETURN_MSG_ALIVE;

    while (running) {
        std::optional<boost::asio::ip::udp::endpoint> current_endpoint;

        {
            std::lock_guard<std::mutex> lock(heartbeat_mutex);
            current_endpoint = heartbeat_endpoint;
        }

        if (current_endpoint) {
            socket.send_to(boost::asio::buffer(&returnData, sizeof(returnData)), *current_endpoint,
                           0, ec);
            if (ec) {
                LOG_F(ERROR, "Failed to send heartbeat: %s", ec.message().c_str());
            }
        }

        // Sleep in smaller increments to allow checking shutdown flag more frequently
        for (int i = 0; i < 100 && running; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

bool GameController::isNewPackage(const RoboCupGameControlData& gcData) {
    /*
     * Here is a corner case. When someone switches from something to init the remaining time is set
     * to 10 minutes so we will detect old packages and will recover after
     * GAMECONTROLLER_MAX_FAULTS_TO_RESET wrong packets. This is not valid in normal games! So this
     * is fine. Don't fix this.
     */

    /* Count faults because there could be a GC reset */
    if (gcData.secsRemaining > lastValidGameControllerTimestamp) {
        LOG_F(ERROR, "Old packet detected. Last received: %u vs %u new received timestamp.",
              lastValidGameControllerTimestamp, gcData.secsRemaining);
        numberOfOldPackagesReceived++;

        if (numberOfOldPackagesReceived < PACKAGES_UNTIL_RESET) {
            return false;
        } else {
            /* We have too many faults. Gamecontroller was may be restarted.
             * We have to initialize our data again */
            lastValidGameControllerTimestamp = gcData.secsRemaining;
            numberOfOldPackagesReceived = 0;
            LOG_F(ERROR, "GameController restart? We reinitialized.");
        }
    } else {
        /* Packet is fine we have to reset our fault counter */
        lastValidGameControllerTimestamp = gcData.secsRemaining;
        numberOfOldPackagesReceived = 0;
    }

    return true;
}
