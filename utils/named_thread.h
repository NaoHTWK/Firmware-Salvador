#pragma once

#include <functional>
#include <string>
#include <thread>

template <typename Callable, typename... Args>
[[nodiscard]] std::thread named_thread(const std::string& name, Callable&& callable,
                                       Args&&... args) {
    auto task = std::bind(std::forward<Callable>(callable), std::forward<Args>(args)...);

    return std::thread([name, task = std::move(task)]() {
        pthread_setname_np(pthread_self(), name.c_str());
        pthread_setschedprio(pthread_self(),
                             sched_get_priority_min(sched_getscheduler(pthread_self())));
        task();
    });
}
