#include <agent_runner.h>
#include <algorithm_ext.h>
#include <dribble.h>
#include <goalie_positioning.h>
#include <localize.h>
#include <logging.h>
#include <motion_pub_sub.h>
#include <search_ball.h>
#include <shoot.h>
#include <stl_ext.h>
#include <walk_relative.h>
#include <walk_to_pos.h>
#include <goalie_dive.h>

#include <future>

#include "team_strategy_pub_sub.h"

AgentRunner::AgentRunner(std::string agent_selection) {
    // Agents registered first trump agents registered later.
    if (agent_selection == "competition") {
        agents.emplace_back(new GoalieDiveAgent);
        agents.emplace_back(new GoaliePositioningAgent);
        agents.emplace_back(new WalkRelativeAgent);
        agents.emplace_back(new LocalizeAgent);
        agents.emplace_back(new BallSearchAgent);
        // agents.emplace_back(new ShootAgent);
        agents.emplace_back(new DribbleAgent);
        agents.emplace_back(new WalkToPositionAgent);
    }
    thread_pool = std::make_unique<ThreadPool>("AgentPool", agents.size());
}

void AgentRunner::proceed() {
    std::shared_ptr<Order> order = team_strategy_order_subscriber.latest();
    std::vector<std::shared_ptr<std::packaged_task<std::tuple<std::string, MotionCommand>()>>>
            agent_futures;
    for (const auto& agent : agents) {
        auto ptr = thread_pool->run<std::tuple<std::string, MotionCommand>>(
                [agent, order]() { return std::make_tuple(agent->name, agent->proceed(order)); });
        agent_futures.push_back(ptr);
    }

    std::vector<std::tuple<std::string, MotionCommand>> agent_results;
    transform_insert(agent_futures, agent_results, [](auto f) { return f->get_future().get(); });

    // Use the first agent which returns something else then Nothing. -> Order of agents decides
    // which agent is executed
    std::tuple<std::string, MotionCommand> result = find_with_default(
            agent_results,
            [](const std::tuple<std::string, MotionCommand>& mc) {
                return std::get<MotionCommand>(mc) != MotionCommand::Nothing;
            },
            std::make_tuple("No Agent", MotionCommand::Nothing));

    log_motion_command(std::get<MotionCommand>(result));
    log_order(*order, std::get<std::string>(result));

    motion_command_channel.publish(std::get<MotionCommand>(result));
}

void AgentRunner::log_motion_command(const MotionCommand& mc) {
    std::string type_str;
    switch (mc.type) {
        case MotionCommand::Type::NOTHING:
            type_str = "NOTHING";
            break;
        case MotionCommand::Type::WALK:
            type_str = "WALK";
            break;
        case MotionCommand::Type::STAND:
            type_str = "STAND";
            break;
        case MotionCommand::Type::JOINT_CONTROL:
            type_str = "JOINT_CONTROL";
            break;
    }

    std::string details;
    if (mc.type == MotionCommand::Type::WALK) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "dx=" << mc.walk_request.dx << " dy=" << mc.walk_request.dy
           << " da=" << mc.walk_request.da;
        details = ss.str();
    } else if (mc.type == MotionCommand::Type::JOINT_CONTROL) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "joints=[";
        for (float j : mc.joint_control_request.joint_control) {
            ss << j << ",";
        }
        ss << "]";
        details = ss.str();
    }

    std::stringstream ss;
    ss << "type=" << type_str << " focus=" << static_cast<int>(mc.focus) << " " << details;
    htwk::log_rerun("behavior/motion_command",
                    rerun::TextLog(rerun::Text(ss.str())).with_level(rerun::TextLogLevel::Debug));
}

void AgentRunner::log_order(const Order& order, std::string agent_name) {
    std::optional<LocPosition> localization = localization_subscriber.latestIfExists();
    if (localization) {
        htwk::log_rerun(
                "soccerfield/robot/order",
                rerun::Points2D({{localization->position.point().x,
                                  localization->position.point().y + 0.5f}})
                        .with_radii(rerun::Radius::ui_points(0.0f))
                        .with_colors(rerun::Color(255, 255, 255, 255))
                        .with_labels(rerun::Text(order.getClassName() + " - " + agent_name)));
    } else {
        htwk::log_rerun(
                "soccerfield/robot/order",
                rerun::Points2D({{-6.f, 0.f}})
                        .with_radii(rerun::Radius::ui_points(0.0f))
                        .with_colors(rerun::Color(255, 255, 255, 255))
                        .with_labels(rerun::Text(order.getClassName() + " - " + agent_name)));
    }
}
