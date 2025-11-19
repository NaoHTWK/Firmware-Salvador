#include "parameter_tuner.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>

ParameterTuner::ParameterTuner(int port) {
    // Create socket
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return;
    }

    // Set socket options
    int opt = 1;
    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        std::cerr << "Error setting socket options" << std::endl;
        return;
    }

    // Bind socket
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_socket, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "Error binding socket" << std::endl;
        return;
    }

    // Listen for connections
    if (listen(server_socket, 3) < 0) {
        std::cerr << "Error listening on socket" << std::endl;
        return;
    }

    // Start socket thread
    try {
        socket_thread = std::thread([this]() { this->socketThread(); });
    } catch (const std::exception& e) {
        std::cerr << "Error creating socket thread: " << e.what() << std::endl;
        close(server_socket);
        server_socket = -1;
    }
}

ParameterTuner::~ParameterTuner() {
    running = false;

    // Close the server socket to unblock accept() call
    if (server_socket >= 0) {
        close(server_socket);
        server_socket = -1;
    }

    if (socket_thread.joinable()) {
        socket_thread.join();
    }
}

float ParameterTuner::getParameter(const std::string& name, float default_value) const {
    std::lock_guard<std::mutex> lock(params_mutex);
    auto it = parameters.find(name);
    return it != parameters.end() ? it->second : default_value;
}

void ParameterTuner::setParameter(const std::string& name, float value) {
    std::lock_guard<std::mutex> lock(params_mutex);
    parameters[name] = value;
}

void ParameterTuner::socketThread() {
    while (running) {
        int client_socket = accept(server_socket, nullptr, nullptr);
        if (client_socket < 0) {
            if (running) {
                std::cerr << "Error accepting connection" << std::endl;
            }
            continue;
        }
        handleClient(client_socket);
        close(client_socket);
    }
}

void ParameterTuner::handleClient(int client_socket) {
    char buffer[1024] = {0};
    while (running) {
        int valread = read(client_socket, buffer, sizeof(buffer) - 1);
        if (valread <= 0)
            break;

        buffer[valread] = '\0';
        parseCommand(buffer);

        // Send acknowledgment
        const char* response = "OK\n";
        send(client_socket, response, strlen(response), 0);
    }
}

void ParameterTuner::parseCommand(const std::string& cmd) {
    // Format: "set param_name value" or "get param_name"
    std::istringstream iss(cmd);
    std::string action, param_name;
    iss >> action >> param_name;

    if (action == "set") {
        float value;
        if (iss >> value) {
            setParameter(param_name, value);
        }
    }
}