#pragma once

#include <atomic>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

class ParameterTuner {
public:
    ParameterTuner(int port = 12345);
    ~ParameterTuner();

    // Delete copy constructor and assignment operator
    ParameterTuner(const ParameterTuner&) = delete;
    ParameterTuner& operator=(const ParameterTuner&) = delete;

    // Get parameter value with thread-safe access
    float getParameter(const std::string& name, float default_value) const;

    // Set parameter value with thread-safe access
    void setParameter(const std::string& name, float value);

private:
    void socketThread();
    void handleClient(int client_socket);
    void parseCommand(const std::string& cmd);

    std::thread socket_thread;
    std::atomic<bool> running{true};
    int server_socket;

    // Thread-safe parameter storage
    mutable std::mutex params_mutex;
    std::unordered_map<std::string, float> parameters;
};