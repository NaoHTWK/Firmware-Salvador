#include <soccerfielddefinitions.h>

SoccerFieldInstance::Parameter getSoccerFieldNaoSpl() {
    return {.fieldLength = 9.f,
            .fieldWidth = 6.f,
            .fieldBorder = 0.5f,
            .circleDiameter = 1.5f,
            .goalPostDistance = 1.5f + 0.1f,
            .goalDepth = 0.475f,
            .penaltySpot2Goal = 1.3f,
            .penaltyAreaWidth = 1.65f,
            .penaltyAreaHeight = 4.f,
            .goalBoxWidth = 0.6f,
            .goalBoxHeight = 2.2f,
            .lineWidth = 0.05f,
            .hasGoalBox = true};
};

SoccerFieldInstance::Parameter getSoccerFieldMiddsize() {
    return {.fieldLength = 22.f,
            .fieldWidth = 14.f,
            .fieldBorder = 0.5f,
            .circleDiameter = 4.f,
            .goalPostDistance = 3.f + 0.1f,
            .goalDepth = 1.f,
            .penaltySpot2Goal = 3.5f,
            .penaltyAreaWidth = 5.f,
            .penaltyAreaHeight = 8.f,
            .goalBoxWidth = 2.f,
            .goalBoxHeight = 5.f,
            .lineWidth = 0.15f,
            .hasGoalBox = true};
};

SoccerFieldInstance::Parameter getSoccerFieldKidSize() {
    return {.fieldLength = 9.f,
            .fieldWidth = 6.f,
            .fieldBorder = 1.f,
            .circleDiameter = 1.5f,
            .goalPostDistance = 2.6f,
            .goalDepth = 0.6f,
            .penaltySpot2Goal = 1.5f,
            .penaltyAreaWidth = 2.f,
            .penaltyAreaHeight = 5.f,
            .goalBoxWidth = 1.f,
            .goalBoxHeight = 3.f,
            .lineWidth = 0.05f,
            .hasGoalBox = true};
};

SoccerFieldInstance::Parameter getSoccerFieldAdultSize() {
    return {.fieldLength = 14.f,
            .fieldWidth = 9.f,
            .fieldBorder = 1.f,
            .circleDiameter = 3.f,
            .goalPostDistance = 2.6f,
            .goalDepth = 0.6f,
            .penaltySpot2Goal = 2.1f,
            .penaltyAreaWidth = 3.f,
            .penaltyAreaHeight = 6.f,
            .goalBoxWidth = 1.f,
            .goalBoxHeight = 4.f,
            .lineWidth = 0.05f,
            .hasGoalBox = true};
};

SoccerFieldInstance::Parameter getSoccerFieldLab() {
    return {.fieldLength = 9.f,
            .fieldWidth = 6.f,
            .fieldBorder = 0.5f,
            .circleDiameter = 1.5f,
            .goalPostDistance = 2.6f + 0.1f,
            .goalDepth = 0.55f,
            .penaltySpot2Goal = 1.3f,
            .penaltyAreaWidth = 1.65f,
            .penaltyAreaHeight = 4.f,
            .goalBoxWidth = 0.6f,
            .goalBoxHeight = 2.2f,
            .lineWidth = 0.05f,
            .hasGoalBox = true};
};

SoccerFieldInstance::Parameter getSoccerFieldPandaEye() {
    return {.fieldLength = 22.f,
            .fieldWidth = 14.f,
            .fieldBorder = 0.9f,
            .circleDiameter = 4.f,
            .goalPostDistance = 2.5f + 0.1f,
            .goalDepth = 0.55f,
            .penaltySpot2Goal = 3.5f,
            .penaltyAreaWidth = 5.f,
            .penaltyAreaHeight = 8.f,
            .goalBoxWidth = 2.f,
            .goalBoxHeight = 5.f,
            .lineWidth = 0.15f,
            .cornerCircleRadius = 0.75f,
            .hasGoalBox = true,
            .hasCornerCircles = true};
};

SoccerFieldInstance::Parameter getSoccerFieldAbuDhabi() {
    return {.fieldLength = 22.f,
            .fieldWidth = 14.f,
            .fieldBorder = 0.9f,
            .circleDiameter = 3.87f,
            .goalPostDistance = 2.5f + 0.1f,
            .goalDepth = 0.55f,
            .penaltySpot2Goal = 3.58f,
            .penaltyAreaWidth = 5.14f,
            .penaltyAreaHeight = 8.f,
            .goalBoxWidth = 2.3f,
            .goalBoxHeight = 5.f,
            .lineWidth = 0.12f,
            .cornerCircleRadius = 0.60f,
            .hasGoalBox = true,
            .hasCornerCircles = true};
};

SoccerFieldInstance::Parameter getSoccerFieldLab75x5() {
    return {.fieldLength = 7.5f,
            .fieldWidth = 5.f,
            .fieldBorder = 0.7f,
            .circleDiameter = 1.5f,
            .goalPostDistance = 1.6f,
            .goalDepth = 0.475f,
            .penaltySpot2Goal = 1.3f,
            .penaltyAreaWidth = 1.65f,
            .penaltyAreaHeight = 3.5f,
            .goalBoxWidth = 0.6f,
            .goalBoxHeight = 2.2f,
            .lineWidth = 0.05f,
            .hasGoalBox = true};
};

SoccerFieldInstance::Parameter getSoccerFieldIFA() {
    return {.fieldLength = 7.5f,
            .fieldWidth = 5.f,
            .fieldBorder = 0.55f,
            .circleDiameter = 1.5f,
            .goalPostDistance = 1.5f + 0.1f,
            .goalDepth = 0.10f,
            .penaltySpot2Goal = 1.3f,
            .penaltyAreaWidth = 1.65f,
            .penaltyAreaHeight = 3.f,
            .goalBoxWidth = 0.6f,
            .goalBoxHeight = 2.2f,
            .lineWidth = 0.05f,
            .hasGoalBox = true};
};
