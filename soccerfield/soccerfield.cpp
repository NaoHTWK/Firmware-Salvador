#include <soccerfield.h>
#include <soccerfielddefinitions.h>

using namespace htwk;

std::vector<point_2d> SoccerField::getGoalPosts() {
    return instance.getGoalPosts();
}

std::vector<Line> SoccerField::getFieldLines() {
    return instance.getFieldLines();
}

std::vector<Line> SoccerField::getBorderLines() {
    return instance.getBorderLines();
}

float SoccerField::circleDiameter() {
    return instance.circleDiameter();
}

std::vector<point_2d> SoccerField::penaltySpots() {
    return instance.getPenaltySpots();
}

float SoccerField::length() {
    return instance.fieldLength();
}

float SoccerField::width() {
    return instance.fieldWidth();
}

float SoccerField::penaltySpot2Goal() {
    return instance.penaltySpot2Goal();
}

float SoccerField::penaltyAreaWidth() {
    return instance.penaltyAreaWidth();
}

float SoccerField::penaltyAreaHeight() {
    return instance.penaltyAreaHeight();
}

float SoccerField::goalBoxWidth() {
    return instance.goalBoxWidth();
}

float SoccerField::goalBoxHeight() {
    return instance.goalBoxHeight();
}

float SoccerField::goalPostDistance() {
    return instance.goalPostDistance();
}

float SoccerField::cornerCircleRadius() {
    return instance.cornerCircleRadius();
}

bool SoccerField::hasCornerCircles() {
    return instance.hasCornerCircles();
}

float SoccerField::fieldLength() {
    return instance.fieldLength();
}

float SoccerField::fieldWidth() {
    return instance.fieldWidth();
}

std::vector<point_2d> SoccerField::get_X() {
    std::vector<point_2d> ret;
    ret.emplace_back(0, instance.circleDiameter() / 2);
    ret.emplace_back(0, 0);
    return ret;
}

std::vector<point_2d> SoccerField::get_T() {
    //ugly fix for the Panda Eye

    std::vector<point_2d> ret;
    // std::vector<point_2d> ret = get_X();
    ret.emplace_back(-instance.fieldLength() / 2, instance.penaltyAreaHeight() / 2);
    ret.emplace_back(-instance.fieldLength() / 2, -instance.penaltyAreaHeight() / 2);
    if (instance.hasGoalBox()) {
        ret.emplace_back(-instance.fieldLength() / 2, instance.goalBoxHeight() / 2);
        ret.emplace_back(-instance.fieldLength() / 2, -instance.goalBoxHeight() / 2);
    }
    ret.emplace_back(0, instance.fieldWidth() / 2);

    if(instance.hasCornerCircles()){
        ret.emplace_back(-instance.fieldLength() / 2, instance.fieldWidth() / 2 - instance.cornerCircleRadius());
        ret.emplace_back(-instance.fieldLength() / 2 + instance.cornerCircleRadius(), instance.fieldWidth() / 2);

        ret.emplace_back(-instance.fieldLength() / 2, -instance.fieldWidth() / 2 + instance.cornerCircleRadius());
        ret.emplace_back(-instance.fieldLength() / 2 + instance.cornerCircleRadius(), -instance.fieldWidth() / 2);
    }
    return ret;
}

std::vector<point_2d> SoccerField::get_L() {
    std::vector<point_2d> ret;
    ret.emplace_back(-instance.fieldLength() / 2, instance.fieldWidth() / 2);
    ret.emplace_back(-instance.fieldLength() / 2, -instance.fieldWidth() / 2);

    ret.emplace_back(-instance.fieldLength() / 2 + instance.penaltyAreaWidth(),
                     instance.penaltyAreaHeight() / 2);
    ret.emplace_back(-instance.fieldLength() / 2 + instance.penaltyAreaWidth(),
                     -instance.penaltyAreaHeight() / 2);

    if (instance.hasGoalBox()) {
        ret.emplace_back(-instance.fieldLength() / 2 + instance.goalBoxWidth(),
                         instance.goalBoxHeight() / 2);
        ret.emplace_back(-instance.fieldLength() / 2 + instance.goalBoxWidth(),
                         -instance.goalBoxHeight() / 2);
    }
    return ret;
}

void SoccerField::set_instance(SoccerFieldInstance new_instance) {
    instance = new_instance;
}

SoccerFieldInstance SoccerField::instance = SoccerFieldInstance({}, 0);