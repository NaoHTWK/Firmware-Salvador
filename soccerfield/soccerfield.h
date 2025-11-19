#pragma once

#include <line.h>
#include <point_2d.h>
#include <soccerfieldinstance.h>

#include <cmath>
#include <vector>

#include "position.h"

class SoccerField {
public:
    static std::vector<point_2d> getGoalPosts();
    static std::vector<htwk::Line> getFieldLines();
    static std::vector<htwk::Line> getBorderLines();
    static std::vector<point_2d> penaltySpots();

    static float length();
    static float width();
    static float circleDiameter();

    static float penaltySpot2Goal();
    static float penaltyAreaWidth();
    static float penaltyAreaHeight();
    static float goalBoxHeight();
    static float goalBoxWidth();
    static float goalPostDistance();

    static point_2d ownGoal() {
        return {-instance.fieldLength() / 2.f, 0.f};
    }
    static point_2d oppGoal() {
        return {instance.fieldLength() / 2.f, 0.f};
    }
    static point_2d ownPenaltySpot() {
        return {-instance.fieldLength() / 2.f + penaltySpot2Goal(), 0.f};
    }
    static bool hasGoalBox() {
        return instance.hasGoalBox();
    }

    static point_2d refereePosition() {
        return instance.getRefereePosition();
    }

    static uint8_t ownFieldside() {
        return instance.getOwnFieldside();
    }

    static const SoccerFieldInstance::Parameter& getParameter() {
        return instance.getParameter();
    }

    static void configureSoccerField(SoccerFieldInstance::Parameter params, bool side) {
        instance = SoccerFieldInstance(params, side);
    }

    static htwk::Line leftSideLine() {
        return {{-length() / 2.f, width() / 2.f}, {length() / 2.f, width() / 2.f}};
    }

    static htwk::Line rightSideLine() {
        return {{-length() / 2.f, -width() / 2.f}, {length() / 2.f, -width() / 2.f}};
    }

    static htwk::Line insideOppGoalLine() {
        return {{length() / 2.f, -goalPostDistance() / 2.f},
                {length() / 2.f, goalPostDistance() / 2.f}};
    }

    // list of all L sections
    static std::vector<point_2d> get_L();

    static std::vector<point_2d> get_T();

    static std::vector<point_2d> get_X();


    static bool hasCornerCircles();
    static float cornerCircleRadius();

    static float fieldLength();
    static float fieldWidth();

    // When standing in the goal, odd jersey numbers are on the left, even on the right,
    // looking into the field at 90 degrees, smaller numbers closer to the goal, spaced
    // evenly.
    static std::vector<htwk::Position> startingPositions() {
        std::vector<htwk::Position> positions;
        for (int i = 0; i < 7; i++) {
            if (i % 2 == 0) {
                positions.emplace_back(-instance.fieldLength() / 2.f / 5.f * (4 - i / 2),
                                       instance.fieldWidth() / 2.f, -M_PIf / 2.f);
            } else {
                positions.emplace_back(-instance.fieldLength() / 2.f / 4.f * (3 - i / 2),
                                       -instance.fieldWidth() / 2.f, M_PIf / 2.f);
            }
        }
        return positions;
    }

    static void set_instance(SoccerFieldInstance new_instance);

private:
    SoccerField() = default;

    static SoccerFieldInstance instance;
};
