#include <boost/interprocess/managed_shared_memory.hpp>
#include <cstdint>
#include <iostream>

#include "shared_memory.h"

// This function is intended to be linked with the robot_process executable
// when running in the simulation environment. It overrides the real-time clock
// function used on the actual robot.

namespace {
// Use a static variable to avoid repeatedly opening shared memory.
// This is a simple optimization.
boost::interprocess::managed_shared_memory& get_segment() {
    static boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only,
                                                              ipc::SharedMemoryName);
    return segment;
}

ipc::GameState* get_game_state() {
    static ipc::GameState* game_state =
            get_segment().find<ipc::GameState>(ipc::GameStateName).first;
    return game_state;
}
}  // namespace

int64_t time_us() {
    ipc::GameState* game_state = get_game_state();
    if (game_state) {
        // In the simulator, time is controlled by the main debugger process
        return game_state->time_us;
    }

    // Fallback or error case.
    // In a real scenario, better error handling might be needed.
    std::cerr << "Warning: Could not get time from shared memory." << std::endl;
    return 0;
}