#include "ready_positioning.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

#include "algorithm_ext.h"
#include "soccerfield.h"
#include "walking_time.h"

using namespace std;

map<size_t, vector<htwk::Position>> readTacticsFile(const string& file) {

    string fileToRead = file;
    if (std::filesystem::exists(file))
        fileToRead = file;
    else if (std::filesystem::exists("tactics/" + file))
        fileToRead = "tactics/" + file;
    else if (std::filesystem::exists("build/tactics/" + file))
        fileToRead = "build/tactics/" + file;
    else if (std::filesystem::exists("/home/booster/etc/tactics/" + file))
        fileToRead = "/home/booster/etc/tactics/" + file;

    ifstream is(fileToRead);
    if (!is.is_open()) {
        printf("Couldn't open team strategy tactics: %s\n", fileToRead.c_str());
        exit(-3);
    }
    static const string rfloat = "([-+]?[0-9]*\\.?[0-9]+)";
    static const string rposition = rfloat + "," + rfloat + "," + rfloat;
    static const regex goalie("GOALY_([0-9]+)=" + rposition);
    static const regex player("POSITION_([0-9]+)_([0-9]+)=" + rposition);

    auto stof = [](const string& s) {
        istringstream iss(s);
        iss.imbue(locale("C"));
        float f;
        iss >> f;
        return f;
    };
    float field_scaling_factor = SoccerField::length() / 9.f;

    map<size_t, vector<htwk::Position>> positions;
    for (string line; getline(is, line);) {
        smatch match_results;
        if (regex_match(line, match_results, goalie)) {
            size_t num_players = static_cast<size_t>(stoi(match_results[1]));
            if (!contains(positions, num_players))
                positions[num_players] = vector<htwk::Position>(num_players + 1);
            positions[num_players][0] = {stof(match_results[2]) * field_scaling_factor,
                                         stof(match_results[3]) * field_scaling_factor,
                                         stof(match_results[4])};
        } else if (regex_match(line, match_results, player)) {
            size_t num_players = static_cast<size_t>(stoi(match_results[1]));
            if (!contains(positions, num_players))
                positions[num_players] = vector<htwk::Position>(num_players + 1u);
            positions[num_players][static_cast<size_t>(stoi(match_results[2]))] = {
                    stof(match_results[3]) * field_scaling_factor,
                    stof(match_results[4]) * field_scaling_factor, stof(match_results[5])};
        }
    }
    for (auto& [num_players, positions] : positions) {
        for (size_t i = 0; i < positions.size(); i++) {
            positions[i].a = point_2d(1, 0).angle_to(point_2d{0.f, 0.f} - positions[i].point());
        }
    }
    return positions;
}

vector<size_t> permutePositions_play(const std::map<PlayerIdx, TeamComData>& robots,
                                     const std::vector<htwk::Position>& positions, bool have_goalie,
                                     std::optional<PlayerIdx> cur_striker) {
    // create a map of all robots and all ready-positions with the distance in walking-time as value
    map<pair<PlayerIdx, size_t /*position_idx*/>, float> distances;
    PlayerIdx striker = cur_striker ? *cur_striker : 255;
    for (auto& [idx, robot] : robots) {
        for (size_t p = 0; p < positions.size(); p++) {
            if (idx != 0 && p == 1 && have_goalie && striker != 0)
                // !goalie && goaly_pos && have_goaly && goaly != striker
                distances[{idx, p}] = numeric_limits<float>::infinity();
            else if (idx == 0 && p != 1 && striker != 0)
                // goalie && !goaly_pos && goalie != striker
                distances[{idx, p}] = numeric_limits<float>::infinity();
            else if (idx != striker && p == 0 && cur_striker)
                // !striker && striker_pos && cur_striker
                distances[{idx, p}] = numeric_limits<float>::infinity();
            else if (idx == striker && p != 0)
                // striker && !striker_pos
                distances[{idx, p}] = numeric_limits<float>::infinity();
            else {
                float walking_time = walkingTime(robot.pos, positions[p]) *
                                     1.4f;  // estimation: slower velocity + avoid obstacle delay
                distances[{idx, p}] = walking_time * walking_time;
            }
        }
    }

    // The runtime for this algorithm is N!, which is horrible. Change once we have more than 6
    // robots. find the smallest overall distance (in walking-time) for all robots to reach a
    // ready-position (1 position can be covered by max 1 robot)
    vector<size_t> position_ids(robots.size());
    iota(position_ids.begin(), position_ids.end(), 0);
    vector<size_t> best_position_ids = position_ids;
    float best_dist = numeric_limits<float>::infinity();
    do {
        float dist = 0;

        size_t i = 0;
        for (auto& [idx, robot] : robots) {
            dist += distances[{idx, position_ids[i++]}];
        }
        if (dist < best_dist) {
            best_dist = dist;
            best_position_ids = position_ids;
        }
    } while (next_permutation(position_ids.begin(), position_ids.end()));

    return best_position_ids;
}

vector<size_t> permutePositions_goaly0(const std::map<PlayerIdx, TeamComData>& robots,
                                       const std::vector<htwk::Position>& positions,
                                       bool have_goalie) {
    // create a map of all robots and all ready-positions with the distance in walking-time as value
    map<pair<PlayerIdx, size_t /*position_idx*/>, float> distances;
    for (const auto& [idx, robot] : robots) {
        for (size_t p = 0; p < positions.size(); p++) {
            if (idx != 0 && p == 0 && have_goalie)
                distances[{idx, p}] = numeric_limits<float>::infinity();
            else if (idx == 0 && p != 0 && have_goalie)
                distances[{idx, p}] = numeric_limits<float>::infinity();
            else {
                // estimation: slower velocity + avoid obstacle delay
                float walking_time = walkingTime(robot.pos, positions[p]);
                distances[{idx, p}] = walking_time;
            }
        }
    }

    if (robots.size() == 1)
        return {0};

    // TODO: The runtime for this algorithm is N!, which is horrible. Since we have 3 set positions
    // (goalie, striker), we can at least reduce the runtime to (N-2)!.
    vector<size_t> position_ids(robots.size());
    iota(position_ids.begin(), position_ids.end(), 0);
    vector<size_t> best_position_ids = position_ids;
    float best_dist = numeric_limits<float>::infinity();
    do {
        float dist = 0;

        size_t i = 0;
        for (auto& [idx, robot] : robots) {
            dist += distances[{idx, position_ids[i++]}];
        }
        if (dist < best_dist) {
            best_dist = dist;
            best_position_ids = position_ids;
        }
    } while (next_permutation(position_ids.begin(), position_ids.end()));
    return best_position_ids;
}

optional<htwk::Position> robocupReadyPositioning(const vector<htwk::Position>& positions_tmp,
                                                 const map<PlayerIdx, TeamComData>& robots,
                                                 PlayerIdx player_idx) {
    vector<htwk::Position> positions(positions_tmp.begin() + 1, positions_tmp.end());
    bool have_goalie = robots.contains(0);
    if (have_goalie && robots.size() > 2)
        positions[0] = positions_tmp[0];

    auto best_position_ids = permutePositions_goaly0(robots, positions, have_goalie);
    size_t i = 0;
    for (const auto& [idx, robot] : robots) {
        if (idx == player_idx) {
            return positions[best_position_ids[i]];
        }
        i++;
    }
    return nullopt;
}
