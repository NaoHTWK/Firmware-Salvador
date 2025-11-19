#pragma once

#include <source_location>

#include "logging.h"
#include "order.h"
#include "point_2d.h"

enum class STRENGTH { LOW, NORMAL };

class ShootOrder : public Order {
public:
    static std::shared_ptr<ShootOrder> create(
            point_2d target, bool allow_passing = false, STRENGTH strength = STRENGTH::NORMAL,
            const std::source_location& loc = std::source_location::current()) {
        auto order =
                std::shared_ptr<ShootOrder>(new ShootOrder(target, allow_passing, strength, loc));
        htwk::log_order(*order,
                        "target: " + std::to_string(target.x) + ", " + std::to_string(target.y) +
                                ", allow_passing: " + std::to_string(allow_passing) +
                                ", strength: " + std::to_string(static_cast<int>(strength)),
                        loc.file_name(), loc.line());
        return order;
    }

    ~ShootOrder() override = default;

    point_2d getTarget() const {
        return target;
    }

    STRENGTH getStrength() const {
        return strength;
    }

    bool passing_allowed() const {
        return allow_passing;
    }

    point_2d target;
    STRENGTH strength;
    bool allow_passing;

private:
    ShootOrder(const ShootOrder&) = delete;
    ShootOrder(ShootOrder&&) = delete;
    ShootOrder& operator=(const ShootOrder&) = delete;
    ShootOrder& operator=(ShootOrder&&) = delete;

    explicit ShootOrder(point_2d target, bool allow_passing = false,
                        STRENGTH strength = STRENGTH::NORMAL,
                        const std::source_location& loc = std::source_location::current())
        : Order("ShootOrder"), target{target}, strength{strength}, allow_passing{allow_passing} {}

    // todo: protobuf?
};
