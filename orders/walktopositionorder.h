#pragma once

#include <source_location>

#include "head_focus.h"
#include "logging.h"
#include "order.h"
#include "position.h"
#include "vector_field.h"

class WalkToPositionOrder : public Order {
public:
    enum class Mode { USE_A, STRIKER, SUPPORTER, FOCUS_DIRECTION };

    static std::shared_ptr<WalkToPositionOrder> create(
            const htwk::Position& pos, Mode mode = Mode::USE_A, bool precise = false,
            std::optional<HeadFocus> head_focus = std::nullopt,
            std::vector<htwk::vectorfield::Influencer> obstacles = {},
            const std::source_location& loc = std::source_location::current()) {
        auto order = std::shared_ptr<WalkToPositionOrder>(
                new WalkToPositionOrder(pos, mode, precise, head_focus, obstacles));
        htwk::log_order(*order,
                        "pos: " + std::to_string(pos.x) + ", " + std::to_string(pos.y) +
                                ", mode: " + std::to_string(static_cast<int>(mode)) +
                                ", precise: " + std::to_string(precise) +
                                ", head_focus: " + (head_focus ? "true" : "false") +
                                ", obstacles: " + std::to_string(obstacles.size()),
                        loc.file_name(), loc.line());
        return order;
    }

    static std::shared_ptr<WalkToPositionOrder> create(
            const point_2d& pos, Mode mode = Mode::USE_A, bool precise = false,
            std::optional<HeadFocus> head_focus = std::nullopt,
            std::vector<htwk::vectorfield::Influencer> obstacles = {},
            const std::source_location& loc = std::source_location::current()) {
        auto order = std::shared_ptr<WalkToPositionOrder>(
                new WalkToPositionOrder(pos, mode, precise, head_focus, obstacles));
        htwk::log_order(*order,
                        "pos: " + std::to_string(pos.x) + ", " + std::to_string(pos.y) +
                                ", mode: " + std::to_string(static_cast<int>(mode)) +
                                ", precise: " + std::to_string(precise) +
                                ", head_focus: " + (head_focus ? "true" : "false") +
                                ", obstacles: " + std::to_string(obstacles.size()),
                        loc.file_name(), loc.line());
        return order;
    }

    const htwk::Position pos;
    const Mode mode;
    const bool precise;
    const std::optional<HeadFocus> head_focus;
    const std::vector<htwk::vectorfield::Influencer> obstacle_influencers;

private:
    WalkToPositionOrder(const point_2d& pos, Mode mode, bool precise = false,
                        std::optional<HeadFocus> head_focus = std::nullopt,
                        std::vector<htwk::vectorfield::Influencer> obstacles = {})
        : Order("WalkToPositionOrder"),
          pos(pos, 0),
          mode(mode),
          precise(precise),
          head_focus(head_focus),
          obstacle_influencers(std::move(obstacles)) {}
    WalkToPositionOrder(htwk::Position pos, Mode mode = Mode::USE_A, bool precise = false,
                        std::optional<HeadFocus> head_focus = std::nullopt,
                        std::vector<htwk::vectorfield::Influencer> obstacles = {})
        : Order("WalkToPositionOrder"),
          pos(pos),
          mode(mode),
          precise(precise),
          head_focus(head_focus),
          obstacle_influencers(std::move(obstacles)) {}
};
