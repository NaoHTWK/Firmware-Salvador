#include <soccerfieldinstance.h>

#include <cmath>
using namespace htwk;

SoccerFieldInstance::SoccerFieldInstance(SoccerFieldInstance::Parameter params, bool side)
    : params(params) {
    float penalty_spot = ((fieldLength() / (2.0f)) - params.penaltySpot2Goal);

    goalPosts.emplace_back(-fieldLength() / 2 - params.lineWidth / 2.f,
                           -params.goalPostDistance / 2);
    goalPosts.emplace_back(-fieldLength() / 2 - params.lineWidth / 2.f,
                           params.goalPostDistance / 2);
    goalPosts.emplace_back(fieldLength() / 2 + params.lineWidth / 2.f,
                           -params.goalPostDistance / 2);
    goalPosts.emplace_back(fieldLength() / 2 + params.lineWidth / 2.f, params.goalPostDistance / 2);
    penaltySpots.emplace_back(penalty_spot, 0.f);
    penaltySpots.emplace_back(-penalty_spot, 0.f);
    // TODO: We often detect penalty spots in the center circle, remove once this is fixed.
    penaltySpots.emplace_back(0.f, 0.f);
    // center line needs to be id 0.
    // center line
    fieldLines.emplace_back(0, -fieldWidth() / 2, 0, fieldWidth() / 2);
    // right side-out line
    fieldLines.emplace_back(-fieldLength() / 2, -fieldWidth() / 2, fieldLength() / 2,
                            -fieldWidth() / 2);
    // left side-out line
    fieldLines.emplace_back(-fieldLength() / 2, fieldWidth() / 2, fieldLength() / 2,
                            fieldWidth() / 2);
    // own goal-out line
    fieldLines.emplace_back(-fieldLength() / 2, -fieldWidth() / 2, -fieldLength() / 2,
                            fieldWidth() / 2);
    // opp goal-out line
    fieldLines.emplace_back(fieldLength() / 2, -fieldWidth() / 2, fieldLength() / 2,
                            fieldWidth() / 2);
    // own penalty area line
    fieldLines.emplace_back(
            -fieldLength() / 2 + params.penaltyAreaWidth, -params.penaltyAreaHeight / 2,
            -fieldLength() / 2 + params.penaltyAreaWidth, params.penaltyAreaHeight / 2);
    // own right penalty area side line
    fieldLines.emplace_back(-fieldLength() / 2, -params.penaltyAreaHeight / 2,
                            -fieldLength() / 2 + params.penaltyAreaWidth,
                            -params.penaltyAreaHeight / 2);
    // own left penalty area side line
    fieldLines.emplace_back(-fieldLength() / 2, params.penaltyAreaHeight / 2,
                            -fieldLength() / 2 + params.penaltyAreaWidth,
                            params.penaltyAreaHeight / 2);
    // opp penalty area line
    fieldLines.emplace_back(
            fieldLength() / 2 - params.penaltyAreaWidth, -params.penaltyAreaHeight / 2,
            fieldLength() / 2 - params.penaltyAreaWidth, params.penaltyAreaHeight / 2);
    // opp left (their left, our right) penalty area side line
    fieldLines.emplace_back(fieldLength() / 2, -params.penaltyAreaHeight / 2,
                            fieldLength() / 2 - params.penaltyAreaWidth,
                            -params.penaltyAreaHeight / 2);
    // opp right (their right, our left)penalty area side line
    fieldLines.emplace_back(fieldLength() / 2, params.penaltyAreaHeight / 2,
                            fieldLength() / 2 - params.penaltyAreaWidth,
                            params.penaltyAreaHeight / 2);
    if (params.hasGoalBox) {
        // own goal box line
        fieldLines.emplace_back(-fieldLength() / 2 + params.goalBoxWidth, -params.goalBoxHeight / 2,
                                -fieldLength() / 2 + params.goalBoxWidth, params.goalBoxHeight / 2);
        // own right goal box side line
        fieldLines.emplace_back(-fieldLength() / 2, -params.goalBoxHeight / 2,
                                -fieldLength() / 2 + params.goalBoxWidth,
                                -params.goalBoxHeight / 2);
        // own left goal box side line
        fieldLines.emplace_back(-fieldLength() / 2, params.goalBoxHeight / 2,
                                -fieldLength() / 2 + params.goalBoxWidth, params.goalBoxHeight / 2);
        // opp goal box line
        fieldLines.emplace_back(fieldLength() / 2 - params.goalBoxWidth, -params.goalBoxHeight / 2,
                                fieldLength() / 2 - params.goalBoxWidth, params.goalBoxHeight / 2);
        // opp left (their left, our right) goal box side line
        fieldLines.emplace_back(fieldLength() / 2, -params.goalBoxHeight / 2,
                                fieldLength() / 2 - params.goalBoxWidth, -params.goalBoxHeight / 2);
        // opp right (their right, our left) goal box side line
        fieldLines.emplace_back(fieldLength() / 2, params.goalBoxHeight / 2,
                                fieldLength() / 2 - params.goalBoxWidth, params.goalBoxHeight / 2);
    }
    // own goal, rectangle where the net is attached
    fieldLines.emplace_back(-fieldLength() / 2, -params.goalPostDistance / 2,
                            -fieldLength() / 2 - params.goalDepth, -params.goalPostDistance / 2);
    fieldLines.emplace_back(-fieldLength() / 2, params.goalPostDistance / 2,
                            -fieldLength() / 2 - params.goalDepth, params.goalPostDistance / 2);
    fieldLines.emplace_back(-fieldLength() / 2 - params.goalDepth, -params.goalPostDistance / 2,
                            -fieldLength() / 2 - params.goalDepth, params.goalPostDistance / 2);
    // opp goal, rectangle where the net is attached
    fieldLines.emplace_back(fieldLength() / 2, -params.goalPostDistance / 2,
                            fieldLength() / 2 + params.goalDepth, -params.goalPostDistance / 2);
    fieldLines.emplace_back(fieldLength() / 2, params.goalPostDistance / 2,
                            fieldLength() / 2 + params.goalDepth, params.goalPostDistance / 2);
    fieldLines.emplace_back(fieldLength() / 2 + params.goalDepth, -params.goalPostDistance / 2,
                            fieldLength() / 2 + params.goalDepth, params.goalPostDistance / 2);

    borderLines.emplace_back(-fieldLength() / 2 - params.fieldBorder,
                             -fieldWidth() / 2 - params.fieldBorder,
                             fieldLength() / 2 + params.fieldBorder,
                             -fieldWidth() / 2 - params.fieldBorder);  // left side-out line
    borderLines.emplace_back(-fieldLength() / 2 - params.fieldBorder,
                             fieldWidth() / 2 + params.fieldBorder,
                             fieldLength() / 2 + params.fieldBorder,
                             fieldWidth() / 2 + params.fieldBorder);  // right side-out line
    borderLines.emplace_back(-fieldLength() / 2 - params.fieldBorder,
                             -fieldWidth() / 2 - params.fieldBorder,
                             -fieldLength() / 2 - params.fieldBorder,
                             fieldWidth() / 2 + params.fieldBorder);  // own goal-out line
    borderLines.emplace_back(fieldLength() / 2 + params.fieldBorder,
                             -fieldWidth() / 2 - params.fieldBorder,
                             fieldLength() / 2 + params.fieldBorder,
                             fieldWidth() / 2 + params.fieldBorder);  // opp goal-out line

    refereePosition = point_2d(0.f, side ? -fieldWidth() / 2 - .25f : fieldWidth() / 2 + .25f);
    ownFieldside = side;
}
