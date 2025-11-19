#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>

// A Queue is a way to pass messages between threads where all samples need to be processed.
// Unlike Channel, which only maintains the latest sample, Queue ensures every published sample
// is delivered to the subscriber. It is designed for situations where the history of samples
// matters, and no samples should be missed.
//
// Example usage:
// processor/queues.h:
//   extern Queue<Event> event_queue;
//
// processor/queues.cpp:
//   #include "processor/queues.h"
//
//   Queue<Event> event_queue;
//
// event_generator.cpp:
//   #include "processor/queues.h"
//   ...
//   event_queue.publish(some_event);
//
// event_processor.cpp:
//   #include "processor/queues.h"
//   ...
//   auto event = event_queue.next(); // Will block until an event is available

// Queue class that guarantees each published sample will be delivered to exactly one subscriber.
// Note: The Queue uses mutexes, so big memcopies will cause all publishers/subscribers to wait. If
//       you want to send big amounts of data through a Queue, use `std::unique_ptr<T>`. Do NOT use
//       `std::shared_ptr<T>`.
// Note: next() can be called from multiple places and each element will be returned exactly once.
template <typename T>
class Queue {
public:
    Queue() = default;

    // Blocks until a sample is available
    T next() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !queue.empty(); });
        T value = std::move(queue.front());
        queue.pop_front();
        return value;
    }

    // Tries to get a value without blocking
    std::optional<T> try_next() {
        std::unique_lock<std::mutex> lock(mtx);
        if (queue.empty()) {
            return std::nullopt;
        }
        T value = std::move(queue.front());
        queue.pop_front();
        return value;
    }

    bool empty() {
        std::unique_lock<std::mutex> lock(mtx);
        return queue.empty();
    }

    size_t size() {
        std::unique_lock<std::mutex> lock(mtx);
        return queue.size();
    }

    void publish(const T& sample) {
        std::unique_lock<std::mutex> lock(mtx);
        queue.push_back(sample);
        cv.notify_all();
    }

    void publish(T&& sample) {
        std::unique_lock<std::mutex> lock(mtx);
        queue.push_back(std::move(sample));
        cv.notify_all();
    }

private:
    std::deque<T> queue;
    std::mutex mtx;
    std::condition_variable cv;
};
