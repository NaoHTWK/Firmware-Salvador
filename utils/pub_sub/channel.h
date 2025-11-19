#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>

// A Channel is a way to pass messages (samples) between threads. It should be used in situations
// where the latest sample is important, rather than the history of samples. It helps to separate
// different components and makes them easier to simulate on their own.
//
// Example usage:
// gamecontroller/channels.h:
//   extern Channel<GameState> game_state;
//
// gamecontroller/channels.cpp:
//   #include "gamecontroller/channels.h"
//
//   Channel<GameState> game_state;
//
// gamecontroller/game_controller.cpp:
//   #include "gamecontroller/channels.h"
//   ...
//   game_state.publish(some_game_state);
//
// team_strategy.cpp:
//   #include "gamecontroller/channels.h"
//   ...
//   auto subscriber = game_state.create_subscriber();
//   ...
//   auto some_game_state = subscriber.next();
//
// gamecontroller/CMakeLists.txt:
//   add_library(gamecontroller_channels channels.cpp channels.h)
//
// team_strategy/CMakeLists.txt:
//   add_executable(team_strategy main.cpp)
//   target_link_libraries(team_strategy PRIVATE gamecontroller_channels)
namespace htwk {
// Note: Subscribers to the same Channel are independent of each other.
template <typename T>

class ChannelSubscriber {
public:
    ChannelSubscriber(std::mutex& mtx, std::condition_variable& cv, const T& sample,
                      const size_t& sample_idx)
        : mtx(mtx), cv(cv), sample(sample), sample_idx(sample_idx) {}

    // Blocks until a sample is available.
    // Note: Useful to make sure there is at least one sample present, so a call to latest() doesn't
    //       return the default-constructed T.
    void waitForInitialSample() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return sample_idx != 0; });
    }

    // Blocks until a new sample is available or returns the latest sample if it hasn't been
    // previously returned by next().
    // Note: Don't assume that next(); /* ... */ next(); will return every sample that was
    //       published. If 2 new samples are published while you're processing the first one, the
    //       2nd call to next(); will return the 2nd new sample and miss the 1st that was published
    //       while you were processing.
    T next() {
        std::unique_lock<std::mutex> lock(mtx);
        if (sample_idx == seen_idx) {
            cv.wait(lock, [this] { return sample_idx != seen_idx; });
        }
        seen_idx = sample_idx;
        return sample;
    }

    // Returns the latest sample without blocking.
    // Note: If no sample has been published yet, returns the default-constructed T.
    T latest() {
        std::unique_lock<std::mutex> lock(mtx);
        return sample;
    }

    // Returns the latest sample without blocking or a default value if no sample has been published
    // yet.
    T latestOr(T&& default_value) {
        std::unique_lock<std::mutex> lock(mtx);
        if (sample_idx == 0) {
            return default_value;
        }
        return sample;
    }

    // Returns the latest sample without blocking or a nullopt if no sample has been published yet.
    std::optional<T> latestIfExists() {
        std::unique_lock<std::mutex> lock(mtx);
        if (sample_idx == 0) {
            return std::nullopt;
        }
        return sample;
    }

private:
    std::mutex& mtx;
    std::condition_variable& cv;
    const T& sample;
    const size_t& sample_idx;
    size_t seen_idx = 0;
};
};  // namespace htwk
namespace htwk {

// Note: Be careful when publishing from multiple places to the same Channel as they will
//       overwrite each others' samples.
template <typename T>
class Channel {
public:
    Channel() {}

    ChannelSubscriber<T> create_subscriber() {
        ChannelSubscriber<T> s{mtx, cv, sample, sample_idx};
        return s;
    }

    void publish(const T& s) {
        std::unique_lock<std::mutex> lock(mtx);
        sample = s;
        sample_idx++;
        cv.notify_all();
    }

private:
    T sample;
    size_t sample_idx = 0;
    std::mutex mtx;
    std::condition_variable cv;
};

};  // namespace htwk
