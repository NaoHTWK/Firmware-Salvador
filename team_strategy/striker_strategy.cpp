#include "striker_strategy.h"

#include <moveballgoalorder.h>
#include <moveballorder.h>
#include <position_util.h>

using namespace htwk;
using namespace std;

void StrikerStrategy::reset() {
    passCandidate = nullopt;
}

shared_ptr<Order> StrikerStrategy::proceed(
        const TeamBall& teamball, const std::map<PlayerIdx, Robot>& alive_robots_,
        const std::multimap<float, ObstacleObservation>& obstacles_by_x) {
    //    const Robot myself = alive_robots_.at(player_idx);

    //    optional<ObstacleObservation> obs_enemy;
    //    for (const auto& obs : obstacles_by_x){
    //        if (obs.first < teamball.pos.x) continue;
    //        if (obs.second.pos.dist(teamball.pos) < 1.f)
    //            obs_enemy = obs.second;
    //    }

    //    if (obs_enemy && teamball.pos.x < 3.f && fabs(teamball.pos.y) < 2.5f &&
    //            Line(teamball.pos, opp_goal).innerDistance(obs_enemy->pos) < .4f){
    //        Line ball_obs_line(teamball.pos, obs_enemy->pos);
    //        float distance_to_opp = 0.5f;
    //        point_2d dodge_opp1 = point_2d(obs_enemy->pos.x + ball_obs_line.u().y*distance_to_opp,
    //                                       obs_enemy->pos.y -
    //                                       ball_obs_line.u().x*distance_to_opp);
    //        point_2d dodge_opp2 = point_2d(obs_enemy->pos.x - ball_obs_line.u().y*distance_to_opp,
    //                                       obs_enemy->pos.y +
    //                                       ball_obs_line.u().x*distance_to_opp);
    //        if (dodge_opp1.dist(opp_goal) < dodge_opp2.dist(opp_goal)){
    //            return make_shared<MoveBallOrder>(dodge_opp1.x, dodge_opp1.y);
    //        }else{
    //            return make_shared<MoveBallOrder>(dodge_opp2.x, dodge_opp2.y);
    //        }make_shared<WalkToPositionOrder>(world_model->getOwnRobot().pos,
    //        WalkToPositionOrder::Mode::SUPPORTER);
    //    }
    return MoveBallGoalOrder::create();
}

shared_ptr<Order> StrikerStrategy::proceed_kickoff(const TeamBall& teamball,
                                                   const std::map<PlayerIdx, Robot>& alive_robots_,
                                                   const vector<ObstacleObservation>& obstacles) {
    alive_robots = alive_robots_;
    if (passCandidate)
        return MoveBallOrder::create(alive_robots[*passCandidate].pos.x + 0.5f,
                                     alive_robots[*passCandidate].pos.y);

    if (alive_robots.size() <= 3 || obstacles.empty()) {
        return MoveBallOrder::create(1.75f, 0.f);
    } else {
        std::multimap<float, PlayerIdx> passCandidate_by_obs_distance;
        for (const auto& bot : alive_robots) {
            if (bot.second.pos.x < -1.f || bot.first == player_idx)
                continue;
            passCandidate_by_obs_distance.emplace(distToClosestObstacle(bot.second, obstacles),
                                                  bot.first);
        }
        passCandidate = passCandidate_by_obs_distance.rbegin()->second;
        return MoveBallOrder::create(alive_robots[*passCandidate].pos.x + 0.5f,
                                     alive_robots[*passCandidate].pos.y);
    }

    //  decide depending on y average
    //    float obs_y_avg=0;
    //    if (obstacles_by_x.empty()){
    //        pass_kickoff = {2.f,0.f};
    //    }else{
    //        for (const auto& obs : obstacles_by_x){
    //            obs_y_avg+=obs.second.pos.y;
    //        }
    //        obs_y_avg/=obstacles_by_x.size();
    //        pass_kickoff = {1.f, (obs_y_avg > 0) ? -2.f : 2.f};
    //    }
    //    return make_shared<MoveBallOrder>(pass_kickoff->x, pass_kickoff->y);
}
