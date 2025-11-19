#pragma once

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <optional>
#include <vector>

#include "gc_state.h"

namespace ipc {

const char* const SharedMemoryName = "TeamStrategyDebugSharedMemory";
constexpr char GameStateName[] = "GameState";
constexpr char RobotStatesName[] = "RobotStates";
constexpr char TickStartSemaphoreName[] = "TickStartSemaphore";
constexpr char TickCompletionSemaphoreName[] = "TickCompletionSemaphore";

constexpr char TickStartBarrierName[] = "TickStartBarrier";
constexpr char TickCompletionBarrierName[] = "TickCompletionBarrier";

constexpr size_t MAX_TEAM_COM_MSG_SIZE = 10240;

// Define types for shared memory allocator and vector
template <typename T>
using ShmemAllocator =
        boost::interprocess::allocator<T,
                                       boost::interprocess::managed_shared_memory::segment_manager>;

struct TeamComMessage {
    size_t data_size;
    char data[MAX_TEAM_COM_MSG_SIZE];
};

struct Vector2f {
    float x;
    float y;
};

struct Pose2D {
    float x;
    float y;
    float a;
};

// The robots are using a RHS coordinate system, so x is forward (towards the opponent's goal), y is
// left.
struct RobotState {
    int id;
    Vector2f position;  // in meters
    float orientation;  // in radians
    bool active = true;
    bool penalized = false;
    std::optional<Pose2D> walk_target;
};

struct GameState {
    ::GameState state = ::GameState::Initial;
    std::optional<Vector2f> ball_position;
    KickingTeam setPlay_kicking_team = KickingTeam::Unknown;
    ::GameState setPlay_state = ::GameState::Initial;
    SecondaryState secondary_state = SecondaryState::Normal;

    int64_t time_us = 0;
    // Add other game state info here, e.g., score, time, etc.

    // Team communication
    boost::interprocess::interprocess_mutex team_com_mutex;

    // These need to be constructed with an allocator
    boost::interprocess::vector<TeamComMessage, ShmemAllocator<TeamComMessage>> team_com_messages;

    GameState(const ShmemAllocator<TeamComMessage>& alloc) : team_com_messages(alloc) {}
};

using TeamComMessageVector =
        boost::interprocess::vector<TeamComMessage, ShmemAllocator<TeamComMessage>>;
using RobotStateVector = boost::interprocess::vector<RobotState, ShmemAllocator<RobotState>>;

class barrier {
public:
    barrier(unsigned int count) : m_count(count), m_generation(0), m_count_down(count) {}

    void wait() {
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(m_mutex);
        unsigned int gen = m_generation;

        if (--m_count_down == 0) {
            m_generation++;
            m_count_down = m_count;
            m_condition.notify_all();
        } else {
            while (gen == m_generation) {
                m_condition.wait(lock);
            }
        }
    }

private:
    barrier(const barrier&) = delete;
    barrier& operator=(const barrier&) = delete;

    boost::interprocess::interprocess_mutex m_mutex;
    boost::interprocess::interprocess_condition m_condition;
    const unsigned int m_count;
    unsigned int m_generation;
    unsigned int m_count_down;
};

}  // namespace ipc