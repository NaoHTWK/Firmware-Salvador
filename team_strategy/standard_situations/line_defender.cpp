#include "line_defender.h"

#include <optional>

#include "keepgoalorder.h"
#include "localization_utils.h"
#include "noorder.h"
#include "ready_positioning.h"
#include "vector_field.h"
#include "walktopositionorder.h"

using namespace std;
using namespace htwk;

shared_ptr<Order> LineDefender::proceedSetPiece(const point_2d& teamball,
                                                const std::map<PlayerIdx, TeamComData>& robots,
                                                const Myself& myself) {
    if (!robots.contains(myself.idx)) {
        return NoOrder::create();
    }
    if (myself.idx == goaly_idx)
        return KeepGoalOrder::create();

    if (!pos) {
        pos = robocupReadyPositioning(selectRole(teamball, robots, myself), robots, myself.idx);
    }

    if (pos->point() == selectPosition(Roles::FRONT_WALL, teamball, myself) && myself.ball) {
        // learning 2023: No change for corner kick, but FRONTWALL changes need to be applied to
        // FRONT_WALL_REL, too
        pos = Position(selectPosition(Roles::FRONT_WALL_REL, teamball, myself), 0.f);
    }
    // Influencer radius is 1.6m and flee is set to make sure to be outside the 1.5m penalty area.
    std::vector<vectorfield::Influencer> obstacles = {vectorfield::Influencer{
            .position = teamball, .radius = 1.6f, .deflection = 1.f, .flee = true}};
    return WalkToPositionOrder::create(*pos, WalkToPositionOrder::Mode::SUPPORTER, false,
                                       std::nullopt, obstacles);
}

vector<Position> LineDefender::selectRole(const point_2d& teamball,
                                          const std::map<PlayerIdx, TeamComData>& robots,
                                          const Myself& myself) {
    vector<Position> positions = vector<Position>(robots.size() + 1);
    switch (robots.size()) {
        case 1:
            positions.at(0) = {
                    selectPosition((teamball.x > thres_corner) ? Roles::FRONT_WALL : Roles::GOALY,
                                   teamball, myself),
                    0.f};
            positions.at(1) = {selectPosition((teamball.x > thres_corner) ? Roles::FRONT_WALL
                                                                          : Roles::BACK_WALL,
                                              teamball, myself),
                               0.f};
            break;
        case 2:
            positions.at(0) = {selectPosition(Roles::GOALY, teamball, myself), 0.f};
            positions.at(1) = {selectPosition(Roles::GOALY, teamball, myself), 0.f};
            positions.at(2) = {selectPosition((teamball.x > thres_corner) ? Roles::FRONT_WALL
                                                                          : Roles::BACK_WALL,
                                              teamball, myself),
                               0.f};
            break;
        case 7:
            positions.at(7) = {selectPosition(Roles::FIELD_COVER, teamball, myself), 0.f};
            /* fall through */
        case 6:
            positions.at(6) = {selectPosition(Roles::COUNTER, teamball, myself), 0.f};
            /* fall through */
        case 5:
            positions.at(5) = {selectPosition(Roles::SIDE_WALL, teamball, myself), 0.f};
            /* fall through */
        case 4:
            positions.at(4) = {selectPosition(Roles::COVER, teamball, myself), 0.f};
            /* fall through */
        case 3:
            positions.at(0) = {selectPosition(Roles::GOALY, teamball, myself), 0.f};
            positions.at(1) = {selectPosition(Roles::GOALY, teamball, myself), 0.f};
            positions.at(2) = {selectPosition(Roles::BACK_WALL, teamball, myself), 0.f};
            positions.at(3) = {selectPosition(Roles::FRONT_WALL, teamball, myself), 0.f};
            break;
    }
    return positions;
}

point_2d LineDefender::selectPosition(Roles role, const point_2d& teamball, const Myself& myself) {
    Line goal_ball_line(own_goal, teamball);
    Line goal_ball_pseudo_line(own_goal, {teamball.x - 1.f, teamball.y});
    const int ball_side = teamball.y > 0 ? 1 : -1;
    point_2d p = myself.loc.position.point();
    switch (role) {
        case Roles::GOALY:
            p = {SoccerField::length() * -.5f + 0.3f, 0.f};
            break;
        case Roles::BACK_WALL:
            p = goal_ball_line.p1() + 1.5f * goal_ball_line.u().normalized();
            break;
        case Roles::FRONT_WALL:
            if (teamball.x < -2.f) {
                p.x = clamped_linear_interpolation(teamball.x, -3.8f, -2.78f, -4.5f, -2.f);
                p.y = clamped_linear_interpolation(teamball.x, 1.2f, 2.08f, -4.5f, -2.f) *
                      ball_side;
            } else {
                p = goal_ball_line.p2() - 1.f * goal_ball_line.u().normalized();
            }
            break;
        case Roles::SIDE_WALL:
            if (teamball.x < -2.f) {
                p.x = clamped_linear_interpolation(teamball.x, -4.f, -3.33f, -4.5f, -2.f);
                p.y = clamped_linear_interpolation(teamball.x, 2.2f, 2.33f, -4.5f, -2.f) *
                      ball_side;
            } else {
                p = goal_ball_pseudo_line.p2() - .75f * goal_ball_pseudo_line.u().normalized();
            }
            break;
        case Roles::COVER:
            p = {-2 + min(1.f, abs(thres_cover - max(thres_cover, teamball.x)) / 2),
                 .5f * ball_side};
            break;
        case Roles::FRONT_WALL_REL:
            if (teamball.x < -2.f) {
                p.x = clamped_linear_interpolation(teamball.x, -3.8f, -2.5f, -4.5f, -2.f);
                p.y = clamped_linear_interpolation(teamball.x, 1.2f, 2.4f, -4.5f, -2.f) * ball_side;
            } else {
                Line goal_relBall_line(own_goal, LocalizationUtils::relToAbs(myself.ball->pos_rel,
                                                                             myself.loc.position));
                p = goal_relBall_line.p2() - .8f * goal_relBall_line.u().normalized();
            }
            break;
        case Roles::COUNTER:
            if (teamball.x < -2.f) {
                p = {-2.f, 2.f * ball_side};
            } else {
                p.x = clamped_linear_interpolation(teamball.x, -2.f, 4.f, -2.0f, 4.5f);
                p.y = 2.f * ball_side;
            }
            break;
        case Roles::FIELD_COVER:  // TODO: nicer uebergang
            if (teamball.x < 1.f) {
                p = {-2.f, .75f * ball_side * -1};
            } else {
                p = goal_ball_line.p2() - 3.5f * goal_ball_line.u().normalized();
            }
            break;
    }
    return p;
}
