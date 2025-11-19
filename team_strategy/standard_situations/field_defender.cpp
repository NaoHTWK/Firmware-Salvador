#include "field_defender.h"

#include <optional>

#include "keepgoalorder.h"
#include "noorder.h"
#include "point_2d.h"
#include "position_util.h"
#include "ready_positioning.h"
#include "vector_field.h"
#include "walktopositionorder.h"

using namespace std;
using placeholders::_1;
using namespace htwk;

shared_ptr<Order> FieldDefender::proceedSetPiece(const point_2d& teamball,
                                                 const std::map<PlayerIdx, TeamComData>& robots,
                                                 const Myself& myself) {
    if (!robots.contains(myself.idx)) {
        return NoOrder::create();
    }

    if (!pos) {
        if (posArea_critical.isWithinArea(teamball)) {
            point_2d p = selectPosition_criticalArea(
                    selectRole_criticalArea(robots, myself.idx, teamball), teamball, robots,
                    myself);
            if (myself.idx == goaly_idx)
                return KeepGoalOrder::create(false, true);//, p);
            pos = Position(p, 0.f);
        } else {
            if (myself.idx == goaly_idx)
                return KeepGoalOrder::create();
            pos = robocupReadyPositioning(selectRole(teamball, robots), robots, myself.idx);
        }
    }

    if (pos->point() == selectPosition(Roles::FRONT_WALL, teamball) && myself.ball)
        pos = Position(selectPosition(Roles::FRONT_WALL_REL, teamball), 0.f);

    // Influencer radius is 1.6m and flee is set to make sure to be outside the 1.5m penalty area.
    std::vector<vectorfield::Influencer> obstacles = {vectorfield::Influencer{
            .position = teamball, .radius = 1.6f, .deflection = 1.f, .flee = true}};
    return WalkToPositionOrder::create(*pos, WalkToPositionOrder::Mode::SUPPORTER, false,
                                       std::nullopt, obstacles);
}

vector<Position> FieldDefender::selectRole(const point_2d& teamball,
                                           const std::map<PlayerIdx, TeamComData>& robots) {
    vector<Position> positions = vector<Position>(robots.size() + 1);
    switch (robots.size()) {
        case 1:
            positions.at(0) = {
                    selectPosition((teamball.x > thres_corner) ? Roles::FRONT_WALL : Roles::GOALY,
                                   teamball),
                    0.f};
            positions.at(1) = {
                    selectPosition((teamball.x > thres_corner) ? Roles::FRONT_WALL : Roles::GOALY,
                                   teamball),
                    0.f};
            break;
        case 2:
            positions.at(0) = {selectPosition(Roles::GOALY, teamball), 0.f};
            positions.at(1) = {selectPosition(Roles::GOALY, teamball), 0.f};
            positions.at(2) = {selectPosition((teamball.x > thres_corner) ? Roles::FRONT_WALL
                                                                          : Roles::BACK_WALL,
                                              teamball),
                               0.f};
            break;
        case 7:
            positions.at(7) = {selectPosition(Roles::BACK_WALL2, teamball), 0.f};
            /* fall through */
        case 6:
            positions.at(6) = {selectPosition(Roles::SIDE_WALL2, teamball), 0.f};
            /* fall through */
        case 5:
            positions.at(5) = {selectPosition(Roles::SIDE_WALL, teamball), 0.f};
            /* fall through */
        case 4:
            positions.at(4) = {selectPosition(Roles::COVER, teamball), 0.f};
            /* fall through */
        case 3:
            positions.at(0) = {selectPosition(Roles::GOALY, teamball), 0.f};
            positions.at(1) = {selectPosition(Roles::GOALY, teamball), 0.f};
            positions.at(2) = {selectPosition(Roles::BACK_WALL, teamball), 0.f};
            positions.at(3) = {selectPosition(Roles::FRONT_WALL, teamball), 0.f};
            break;
    }

    // sort(positions.begin(), positions.end(), compareByX);
    return positions;
}

point_2d FieldDefender::selectPosition(Roles role, const point_2d& teamball) {
    const int ball_side = teamball.y > 0 ? 1 : -1;
    Line goal_ball_line(own_goal, teamball);
    Line goal_ball_pseudo_left_line(own_goal, {teamball.x, teamball.y + .75f * ball_side});
    point_2d p;
    float offset;
    switch (role) {
        case Roles::GOALY:
            return {SoccerField::length() * -.5f + 0.3f, 0.f};
        case Roles::BACK_WALL:
            p = goal_ball_line.p1() + 1.5f * goal_ball_line.u().normalized();
            if (p.dist(teamball) < 1.f)
                p = goal_ball_line.p2() - 1.f * goal_ball_line.u().normalized();
            return p;
        case Roles::FRONT_WALL:
            return goal_ball_line.p2() - 1.f * goal_ball_line.u().normalized();
        case Roles::SIDE_WALL:
            return goal_ball_pseudo_left_line.p2() -
                   1.75f * goal_ball_pseudo_left_line.u().normalized();
        case Roles::COVER:
            return {-2 + min(1.f, abs(thres_cover - max(thres_cover, teamball.x)) / 2),
                    -.5f * ball_side};
        case Roles::CRITICAL:
            break;
        case Roles::BACK_WALL2:
            p = goal_ball_line.p1() + 1.5f * goal_ball_line.u().normalized();
            if (p.dist(teamball) < 1.f)
                p = goal_ball_line.p2() - 1.f * goal_ball_line.u().normalized();
            offset = (p.y >= 0) ? 1.f : -1.f;
            return point_2d(p.x + 0.5f, p.y + offset);
        case Roles::SIDE_WALL2:
            p = goal_ball_pseudo_left_line.p2() -
                1.75f * goal_ball_pseudo_left_line.u().normalized();
            offset = (teamball.y > p.y) ? 0.75f : -0.75f;
            return point_2d(p.x - 0.5f, clamp(p.y + offset, -fieldwidth_offset, fieldwidth_offset));
        case Roles::FRONT_WALL_REL:
            Line goal_relBall_line(own_goal, teamball);
            return goal_relBall_line.p2() - .8f * goal_relBall_line.u().normalized();
    }
    return {0.f, 0.f};
}

FieldDefender::Roles FieldDefender::selectRole_criticalArea(
        const std::map<PlayerIdx, TeamComData>& robots, PlayerIdx player_idx,
        const point_2d& teamball) {

    const PlayerIdx near_to_goaly =
            robots.contains(goaly_idx) ? goaly_idx : nearestPlayerToLine(own_goal_line, robots);
    if (player_idx == near_to_goaly)
        return Roles::GOALY;

    const point_2d bwall_pos = selectPosition(Roles::BACK_WALL, teamball);
    const PlayerIdx near_to_bwall = nearestFieldPlayerToPoint(bwall_pos, {near_to_goaly}, robots);
    if (bwall_pos.dist(robots.at(near_to_goaly).pos.point()) > .75f && player_idx == near_to_bwall)
        return Roles::BACK_WALL;

    const point_2d cover_pos = selectPosition(Roles::COVER, teamball);
    const PlayerIdx near_to_cover =
            nearestFieldPlayerToPoint(cover_pos, {near_to_goaly, near_to_bwall}, robots);
    if (robots.size() > 3 && player_idx == near_to_cover)
        return Roles::COVER;

    return Roles::CRITICAL;
}

point_2d FieldDefender::selectPosition_criticalArea(Roles role, const point_2d& teamball,
                                                    const std::map<PlayerIdx, TeamComData>& robots,
                                                    const Myself& myself) {
    switch (role) {
        case Roles::GOALY:
            my_walkToPos_history.clear();
            return {SoccerField::length() * -.5f + .2f, clamp(teamball.y * .75f, -.9f, .9f)};
        case Roles::BACK_WALL:
            my_walkToPos_history.clear();
            return selectPosition(Roles::BACK_WALL, teamball);
        case Roles::COVER:
            my_walkToPos_history.clear();
            return selectPosition(Roles::COVER, teamball);
        case Roles::BACK_WALL2:
            /* fall through */
        case Roles::SIDE_WALL2:
            /* fall through */
        case Roles::FRONT_WALL_REL:
            /* fall through */
        case Roles::FRONT_WALL:
            /* fall through */
        case Roles::SIDE_WALL:
            /* fall through */
        case Roles::CRITICAL:
            my_walkToPos_history.push_back(generatePosition(
                    posArea_move,
                    bind_front(&FieldDefender::getCriticalScore, this, teamball, robots, myself)));
            return smoothPosition(my_walkToPos_history, 80);
    }
    return {0.f, 0.f};
}

float FieldDefender::getCriticalScore(const point_2d& teamball,
                                      const std::map<PlayerIdx, TeamComData>& robots,
                                      const Myself& myself, const point_2d& sim_pos) {
    optional<float> inner_distance =
            Line(myself.loc.position.point(), sim_pos).innerDistance(teamball);
    if (sim_pos.dist(teamball) < 1.2f || inner_distance ? inner_distance < 1.f : false)
        return numeric_limits<float>::infinity();

    if (playerWithinArea(robots, myself.idx, posArea_goal) > 1 &&
        (posArea_goal.isWithinArea(sim_pos) ||
         posArea_goal.isCrossingArea({myself.loc.position.point(), sim_pos})))
        return numeric_limits<float>::infinity();

    float dist_to_neighbor =
            max(0.f, .75f - distToClosestNeighbor(sim_pos, robots, myself.idx)) / .75f;
    float dist_to_ball = sim_pos.dist(teamball);
    float too_far_to_walk = sim_pos.dist(myself.loc.position.point());
    float dist_to_goal = sim_pos.dist(own_goal);

    return 2 * dist_to_neighbor + 2 * dist_to_ball + dist_to_goal + too_far_to_walk;
}
