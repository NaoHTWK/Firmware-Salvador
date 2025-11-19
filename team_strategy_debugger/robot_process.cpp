#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/serialization/vector.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <typeinfo>

#include "channel.h"
#include "gc_pub_sub.h"
#include "keepgoalorder.h"
#include "localization_pub_sub.h"
#include "localization_utils.h"
#include "moveballgoalorder.h"
#include "multi_target_tracker_pub_sub.h"
#include "order.h"
#include "point_2d.h"
#include "position.h"
#include "robocup_strategy.h"
#include "sensor_pub_sub.h"
#include "shared_memory.h"
#include "soccerfield.h"
#include "soccerfielddefinitions.h"
#include "tc_network_serialization.h"
#include "tc_pub_sub.h"
#include "walktopositionorder.h"

using boost_lock = boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex>;

void simulate_walk(ipc::RobotState& robot_state, const htwk::Position& target_pos_htwk) {
    const float time_step_s = 1.f / 30.f;
    const float speed_mps = 0.5f;
    const float angular_speed_radps = 1.5f;
    const float max_dist_delta = speed_mps * time_step_s;
    const float max_angle_delta = angular_speed_radps * time_step_s;

    const ipc::Vector2f target_pos = {target_pos_htwk.x, target_pos_htwk.y};
    const float target_orientation = target_pos_htwk.a;

    const float dx = target_pos.x - robot_state.position.x;
    const float dy = target_pos.y - robot_state.position.y;
    const float dist = std::hypot(dx, dy);

    // Use turn-walk-turn for distances > 0.8m, omni-walk for shorter distances.
    if (dist > 1.25f) {  // Turn-walk-turn
        const float angle_to_target = std::atan2(dy, dx);
        float angle_diff = angle_to_target - robot_state.orientation;
        angle_diff = normalizeRotation(angle_diff);

        const float orientation_tolerance = 5.0f * M_PI / 180.0f;  // 5 degrees

        if (std::abs(angle_diff) > orientation_tolerance) {
            // 1. Turn towards target point
            robot_state.orientation += clamp(angle_diff, -max_angle_delta, max_angle_delta);
        } else if (dist > 0.01f) {
            // 2. Walk straight to target point
            robot_state.orientation = angle_to_target;  // Face target while walking
            if (dist > max_dist_delta) {
                robot_state.position.x += (dx / dist) * max_dist_delta;
                robot_state.position.y += (dy / dist) * max_dist_delta;
            } else {
                robot_state.position.x = target_pos.x;
                robot_state.position.y = target_pos.y;
            }
        } else {
            // 3. Turn to final orientation
            float final_angle_diff = target_orientation - robot_state.orientation;
            final_angle_diff = normalizeRotation(final_angle_diff);
            robot_state.orientation += clamp(final_angle_diff, -max_angle_delta, max_angle_delta);
        }
    } else {  // Omni-directional walk
        // Position
        if (dist > max_dist_delta) {
            robot_state.position.x += (dx / dist) * max_dist_delta;
            robot_state.position.y += (dy / dist) * max_dist_delta;
        } else {
            robot_state.position.x = target_pos.x;
            robot_state.position.y = target_pos.y;
        }

        // Orientation
        float angle_diff = target_orientation - robot_state.orientation;
        angle_diff = normalizeRotation(angle_diff);
        robot_state.orientation += clamp(angle_diff, -max_angle_delta, max_angle_delta);
    }

    robot_state.orientation = normalizeRotation(robot_state.orientation);
}

void process_order(std::shared_ptr<Order> order, ipc::RobotState& robot_state,
                   ipc::GameState& game_state) {
    robot_state.walk_target = std::nullopt;
    if (!order)
        return;

    if (auto* keep_goal = dynamic_cast<KeepGoalOrder*>(order.get())) {
        order = WalkToPositionOrder::create(htwk::Position{-SoccerField::length() / 2 + 0.3f, 0, 0},
                                            WalkToPositionOrder::Mode::USE_A);
    }

    // Simplified "Fake Walk to Pos Agent"
    if (auto* walk_order = dynamic_cast<WalkToPositionOrder*>(order.get())) {
        robot_state.walk_target =
                ipc::Pose2D{walk_order->pos.x, walk_order->pos.y, walk_order->pos.a};
        simulate_walk(robot_state, walk_order->pos);
    } else if (auto* move_ball_order = dynamic_cast<MoveBallGoalOrder*>(order.get())) {
        const float ball_dist_threshold = 0.2f;
        ipc::Vector2f ball_pos = game_state.ball_position.value();

        float dx_ball = ball_pos.x - robot_state.position.x;
        float dy_ball = ball_pos.y - robot_state.position.y;
        float dist_to_ball = std::sqrt(dx_ball * dx_ball + dy_ball * dy_ball);

        if (dist_to_ball < ball_dist_threshold) {
            const float opponent_goal_x = SoccerField::length() / 2.f;
            const float opponent_goal_y = 0.f;
            const float kick_distance = 1.0f;

            ipc::Vector2f goal_dir = {opponent_goal_x - ball_pos.x, opponent_goal_y - ball_pos.y};
            float goal_dist = std::sqrt(goal_dir.x * goal_dir.x + goal_dir.y * goal_dir.y);
            if (goal_dist > 0) {
                goal_dir.x /= goal_dist;
                goal_dir.y /= goal_dist;
            }

            game_state.ball_position->x += goal_dir.x * kick_distance;
            game_state.ball_position->y += goal_dir.y * kick_distance;
        } else {
            htwk::Position target{ball_pos.x, ball_pos.y, std::atan2(dy_ball, dx_ball)};
            simulate_walk(robot_state, target);
        }
    }
    // TODO: Add handlers for other order types, especially MoveBallGoalOrder.
    // TODO: First move the robot "behind" the ball and then "push" it.
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot_id>" << std::endl;
        return 1;
    }

    int robot_id = -1;
    try {
        robot_id = std::stoi(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Invalid robot ID: " << argv[1] << std::endl;
        return 1;
    }
    SoccerField::configureSoccerField(getSoccerFieldPandaEye(), 0);

    std::cout << "Robot " << robot_id << " started." << std::endl;

    try {
        // Open the existing shared memory segment
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only,
                                                           ipc::SharedMemoryName);

        // Find the GameState and RobotStateVector
        auto* game_state_ipc = segment.find<ipc::GameState>(ipc::GameStateName).first;
        auto* robot_states_ipc = segment.find<ipc::RobotStateVector>(ipc::RobotStatesName).first;
        auto* start_barrier = segment.find<ipc::barrier>(ipc::TickStartBarrierName).first;
        auto* completion_barrier = segment.find<ipc::barrier>(ipc::TickCompletionBarrierName).first;

        if (!game_state_ipc || !robot_states_ipc || robot_id >= robot_states_ipc->size()) {
            std::cerr << "Robot " << robot_id
                      << ": Could not find shared memory objects or ID is out of bounds."
                      << std::endl;
            return 1;
        }

        if (!start_barrier || !completion_barrier) {
            std::cerr << "Robot " << robot_id << ": Could not find barriers in shared memory."
                      << std::endl;
            return 1;
        }

        ipc::RobotState& robot_state = (*robot_states_ipc)[robot_id];

        // --- Pub/Sub Mocking ---
        // Channels are defined as global variables in the respective pub_sub libraries.
        // We just need to include the headers and then we can use the channels directly.

        // TODO: simulate the team com broadcast and receive functionality: write the data sent by
        // the robot to shared memory and provide it to the other robots before the next sim loop.

        // --- Strategy Initialization ---
        RobocupStrategy strategy(13, robot_id);
        htwk::ChannelSubscriber<std::shared_ptr<Order>> team_strategy_order_subscriber =
                team_strategy_order_channel.create_subscriber();

        while (robot_state.active) {

            // --- Team Com Receive ---
            {
                boost_lock lock(game_state_ipc->team_com_mutex);

                for (const auto& msg : game_state_ipc->team_com_messages) {
                    TeamComData rcv_data;
                    std::stringstream ss;
                    ss.write(msg.data, msg.data_size);
                    boost::archive::binary_iarchive ia(ss);
                    ia >> rcv_data;
                    tc_internal::receive(rcv_data);
                }
            }

            // --- 1. Read from Shared Memory & Publish to Mock Channels ---

            start_barrier->wait();
            {
                boost_lock lock(game_state_ipc->team_com_mutex);
                game_state_ipc->team_com_messages.clear();

                // Publish GameController state (mostly default values)
                GCState gc_data;
                gc_data.state = game_state_ipc->state;
                for (size_t i = 0; i < robot_states_ipc->size(); ++i) {
                    gc_data.my_team.players.push_back(Player{(*robot_states_ipc)[i].penalized});
                }
                gc_data.secondary_time = 45;
                gc_data.secs_remaining = 500;
                gc_data.secondary_state = game_state_ipc->secondary_state;
                gc_data.kicking_team = KickingTeam::MyTeam;
                gc_data.setPlay_kicking_team = game_state_ipc->setPlay_kicking_team;
                gc_data.setPlay_state = game_state_ipc->setPlay_state;
                gc_state.publish(gc_data);
                // TODO: Fill in the players of our team and mark them as not penalized.

                // Publish this robot's position
                LocPosition loc_pos;
                loc_pos.position = {robot_state.position.x, robot_state.position.y,
                                    robot_state.orientation};
                loc_pos.quality = 0.9f;
                loc_position_channel.publish(loc_pos);

                // Publish fallen state
                htwk::fallen_channel.publish({});

                // Publish relative ball position
                if (std::optional<ipc::Vector2f> ball_pos_abs = game_state_ipc->ball_position) {
                    RelBall rel_ball;
                    rel_ball.pos_rel = LocalizationUtils::absToRel(
                            {ball_pos_abs->x, ball_pos_abs->y},
                            {robot_state.position.x, robot_state.position.y,
                             robot_state.orientation});
                    rel_ball_channel.publish(rel_ball);
                } else {
                    rel_ball_channel.publish(std::nullopt);
                }
            }
            strategy.proceed();
            std::shared_ptr<Order> order = team_strategy_order_subscriber.latest();
            // if (order) {
            //     if (dynamic_cast<WalkToPositionOrder*>(order.get())) {
            //         std::cout << "Robot " << robot_id << " received order: WalkToPositionOrder"
            //                   << std::endl;
            //     } else if (dynamic_cast<MoveBallGoalOrder*>(order.get())) {
            //         std::cout << "Robot " << robot_id << " received order: MoveBallGoalOrder"
            //                   << std::endl;
            //     } else {
            //         std::cout << "Robot " << robot_id
            //                   << " received an unknown order type: " << typeid(*order).name()
            //                   << std::endl;
            //     }
            // } else {
            //     std::cout << "Robot " << robot_id << " received null order" << std::endl;
            // }
            process_order(order, robot_state, *game_state_ipc);
            completion_barrier->wait();

            // --- 4. Handle team communication ---
            while (auto data = tc_internal::broadcast.try_next()) {
                std::stringstream ss;
                boost::archive::binary_oarchive oa(ss);
                oa << *data;
                std::string serialized_str = ss.str();

                ipc::TeamComMessage msg;
                msg.data_size = serialized_str.size();
                std::memcpy(msg.data, serialized_str.c_str(), msg.data_size);

                {
                    boost_lock lock(game_state_ipc->team_com_mutex);
                    game_state_ipc->team_com_messages.push_back(msg);
                }
            }
        }

    } catch (const boost::interprocess::interprocess_exception& ex) {
        std::cerr << "Robot " << robot_id << " - Interprocess exception: " << ex.what()
                  << std::endl;
        return 1;
    } catch (const std::exception& ex) {
        std::cerr << "Robot " << robot_id << " - Exception: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}