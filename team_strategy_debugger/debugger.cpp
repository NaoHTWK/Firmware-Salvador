#include <GLFW/glfw3.h>

#include <boost/asio/io_context.hpp>
#include <boost/filesystem.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/process/v2/process.hpp>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "gc_state.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "shared_memory.h"
#include "soccer_field_visualizer.h"
#include "soccerfield.h"
#include "soccerfielddefinitions.h"

constexpr int NUM_ROBOTS = 5;

struct ShmemCleanup {
    ShmemCleanup() {
        boost::interprocess::shared_memory_object::remove(ipc::SharedMemoryName);
    }
    ~ShmemCleanup() {
        boost::interprocess::shared_memory_object::remove(ipc::SharedMemoryName);
    }
};

static void glfw_error_callback(int error, const char* description) {
    std::cerr << "Glfw Error " << error << ": " << description << std::endl;
}

int main(int argc, char* argv[]) {
    int debug_robot_id = -1;
    if (argc > 2 && std::string(argv[1]) == "--debug-robot") {
        try {
            debug_robot_id = std::stoi(argv[2]);
        } catch (const std::exception& e) {
            std::cerr << "Invalid robot ID for debugging: " << argv[2] << std::endl;
            return 1;
        }
    }

    // Get the path to the current executable
    boost::filesystem::path exe_path =
            boost::filesystem::canonical(boost::filesystem::path(argv[0]));
    boost::filesystem::path exe_dir = exe_path.parent_path();
    ShmemCleanup shmem_cleanup;

    SoccerField::configureSoccerField(getSoccerFieldPandaEye(), 0);

    boost::interprocess::managed_shared_memory segment(boost::interprocess::create_only,
                                                       ipc::SharedMemoryName, 65536);
    const ipc::ShmemAllocator<ipc::TeamComMessage> tc_alloc(segment.get_segment_manager());
    ipc::GameState* game_state = segment.construct<ipc::GameState>(ipc::GameStateName)(tc_alloc);
    const ipc::ShmemAllocator<ipc::RobotState> alloc_inst(segment.get_segment_manager());
    ipc::RobotStateVector* robot_states =
            segment.construct<ipc::RobotStateVector>(ipc::RobotStatesName)(alloc_inst);

    const unsigned int barrier_count = NUM_ROBOTS + 1;
    segment.construct<ipc::barrier>(ipc::TickStartBarrierName)(barrier_count);
    segment.construct<ipc::barrier>(ipc::TickCompletionBarrierName)(barrier_count);

    game_state->state = GameState::Initial;
    game_state->time_us = 0;
    const auto start_positions = SoccerField::startingPositions();
    for (int i = 0; i < NUM_ROBOTS; ++i) {
        if (i >= start_positions.size()) {
            std::cerr << "Warning: Not enough starting positions for all robots." << std::endl;
            break;
        }
        ipc::RobotState state;
        state.id = i;
        const auto& start_pos = start_positions[i];
        state.position = {start_pos.x, start_pos.y};
        state.orientation = start_pos.a;
        state.active = true;
        robot_states->push_back(state);
    }

    boost::asio::io_context ctx;
    std::vector<boost::process::v2::process> children;
    for (int i = 0; i < NUM_ROBOTS; ++i) {
        boost::filesystem::path robot_process_path = exe_dir / "robot_process";
        if (i == debug_robot_id) {
            children.emplace_back(
                    ctx, "/usr/bin/xterm",
                    std::vector<std::string>{"-e", "gdb", "--args", robot_process_path.string(),
                                             std::to_string(i)});
        } else {
            children.emplace_back(ctx, robot_process_path,
                                  std::vector<std::string>{std::to_string(i)});
        }
    }

    std::thread io_thread([&ctx] { ctx.run(); });

    constexpr int64_t sim_time_step_us = 1'000'000 / 30;
    float sim_time_factor = 1.0;
    bool paused = true;
    int64_t last_sim_time_step_us = 0;
    bool shutdown_sim = false;
    std::mutex sim_time_mutex;
    (*robot_states)[0].penalized = true;

    bool is_dragging_ball = false;

    std::thread sim_thread([&]() {
        auto* start_barrier = segment.find<ipc::barrier>(ipc::TickStartBarrierName).first;
        auto* completion_barrier = segment.find<ipc::barrier>(ipc::TickCompletionBarrierName).first;

        if (!start_barrier || !completion_barrier) {
            std::cerr << "Could not find barriers in shared memory." << std::endl;
            shutdown_sim = true;
            return;
        }

        while (!shutdown_sim) {
            std::unique_lock<std::mutex> lock(sim_time_mutex);
            if (paused) {
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::microseconds(sim_time_step_us));
                continue;
            }
            last_sim_time_step_us += (int64_t)(sim_time_step_us / sim_time_factor);
            int64_t next_sim_time_step_us = last_sim_time_step_us;
            lock.unlock();

            start_barrier->wait();  // Wait for all robots to be ready for the tick

            // All robots are ready, update game state
            game_state->time_us += sim_time_step_us;

            completion_barrier->wait();  // Wait for all robots to finish the tick

            // All robots are done, sleep until next tick
            std::this_thread::sleep_until(std::chrono::system_clock::time_point(
                    std::chrono::microseconds(next_sim_time_step_us)));
        }
    });

    SoccerFieldVisualizer visualizer("Team Strategy Debugger");

    while (!visualizer.windowShouldClose()) {
        visualizer.beginFrame();

        ImGui::Begin("Controls");
        if (ImGui::Button(paused ? "Resume" : "Pause")) {
            std::lock_guard<std::mutex> lock(sim_time_mutex);
            paused = !paused;
            if (!paused) {
                last_sim_time_step_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                                std::chrono::system_clock::now().time_since_epoch())
                                                .count();
            }
        }
        ImGui::SliderFloat("Sim Speed", &sim_time_factor, 0.1f, 10.0f);
        ImGui::Text("Sim time: %.2f s", game_state->time_us / 1'000'000.0);
        ImGui::End();

        ImGui::Begin("Game State Controls");
        if (ImGui::Button("Initial")) {
            std::lock_guard<std::mutex> lock(sim_time_mutex);
            game_state->state = GameState::Initial;
            game_state->ball_position = std::nullopt;
            for (int i = 0; i < NUM_ROBOTS; ++i) {
                const auto& start_pos = start_positions[i];
                (*robot_states)[i].position = {start_pos.x, start_pos.y};
                (*robot_states)[i].orientation = start_pos.a;
            }
            game_state->secondary_state = SecondaryState::Normal;
        }
        ImGui::SameLine();
        if (ImGui::Button("Ready")) {
            std::lock_guard<std::mutex> lock(sim_time_mutex);
            game_state->state = GameState::Ready;
            game_state->ball_position = std::nullopt;
            game_state->secondary_state = SecondaryState::Normal;
        }
        ImGui::SameLine();
        if (ImGui::Button("Set")) {
            std::lock_guard<std::mutex> lock(sim_time_mutex);
            game_state->state = GameState::Set;
            game_state->ball_position = {0.f, 0.f};
            game_state->secondary_state = SecondaryState::Normal;
        }
        ImGui::SameLine();
        if (ImGui::Button("Playing")) {
            std::lock_guard<std::mutex> lock(sim_time_mutex);
            game_state->state = GameState::Playing;
            game_state->secondary_state = SecondaryState::Normal;
        }
        if (ImGui::Button("SP Initial")) {
            std::lock_guard<std::mutex> lock(sim_time_mutex);
            game_state->state = GameState::Playing;
            game_state->secondary_state = SecondaryState::DirectFreeKick;
            game_state->setPlay_state = GameState::Initial;
        }
        ImGui::SameLine();
        if (ImGui::Button("SP Ready")) {
            std::lock_guard<std::mutex> lock(sim_time_mutex);
            game_state->state = GameState::Playing;
            game_state->setPlay_state = GameState::Ready;
        }
        ImGui::SameLine();
        if (ImGui::Button("SP Set")) {
            std::lock_guard<std::mutex> lock(sim_time_mutex);
            game_state->state = GameState::Playing;
            game_state->setPlay_state = GameState::Set;
        }
        if (ImGui::Button("SP Own")) {
            std::lock_guard<std::mutex> lock(sim_time_mutex);
            game_state->setPlay_kicking_team = KickingTeam::MyTeam;
        }
        ImGui::SameLine();
        if (ImGui::Button("SP Opp")) {
            std::lock_guard<std::mutex> lock(sim_time_mutex);
            game_state->setPlay_kicking_team = KickingTeam::OppTeam;
        }
        ImGui::End();

        // --- Ball Dragging ---
        if (is_dragging_ball) {
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                is_dragging_ball = false;
            } else {
                std::lock_guard<std::mutex> lock(sim_time_mutex);
                if (game_state->ball_position) {
                    point_2d new_ball_pos_field = visualizer.fromScreen(ImGui::GetMousePos());
                    game_state->ball_position->x = new_ball_pos_field.x;
                    game_state->ball_position->y = new_ball_pos_field.y;
                }
            }
        } else if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !ImGui::IsAnyItemActive() &&
                   !ImGui::IsAnyItemHovered()) {
            if (game_state->ball_position) {
                ImVec2 ball_pos_screen = visualizer.toScreen(game_state->ball_position->x,
                                                             game_state->ball_position->y);
                float ball_radius_screen = 0.15f * visualizer.getScale();
                ImVec2 mouse_pos = ImGui::GetMousePos();
                float dist_sq =
                        (mouse_pos.x - ball_pos_screen.x) * (mouse_pos.x - ball_pos_screen.x) +
                        (mouse_pos.y - ball_pos_screen.y) * (mouse_pos.y - ball_pos_screen.y);
                if (dist_sq < ball_radius_screen * ball_radius_screen) {
                    is_dragging_ball = true;
                }
            }
        }

        // --- Drawing ---
        std::vector<RobotDrawInfo> robot_draw_infos;
        for (const auto& robot_state : *robot_states) {
            robot_draw_infos.push_back(RobotDrawInfo{
                    .id = robot_state.id,
                    .active = robot_state.active,
                    .pos = htwk::Position{robot_state.position.x, robot_state.position.y,
                                          robot_state.orientation},
                    .walk_target = robot_state.walk_target ? std::make_optional<htwk::Position>(
                                                                     robot_state.walk_target->x,
                                                                     robot_state.walk_target->y,
                                                                     robot_state.walk_target->a)
                                                           : std::nullopt});
        }
        visualizer.drawFieldAndRobots(
                robot_draw_infos, game_state->ball_position ? std::make_optional<point_2d>(
                                                                      game_state->ball_position->x,
                                                                      game_state->ball_position->y)
                                                            : std::nullopt);

        // --- Rendering ---
        visualizer.endFrame();
    }

    {
        std::unique_lock<std::mutex> lock(sim_time_mutex);
        shutdown_sim = true;
        lock.unlock();
        sim_thread.join();
    }

    // Terminate robot processes
    for (auto& child : children) {
        if (child.running()) {
            child.terminate();
        }
    }

    io_thread.join();

    // Wait for all processes to finish
    for (auto& child : children) {
        if (child.running()) {
            child.wait();
        }
    }

    return 0;
}