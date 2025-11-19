#pragma once

// NOTHING: Let HeadControl decide where to look.
// LOC: Try to improve localization.
// PREFER_BALL: Look at the ball, but if loc is bad look elsewhere for short periods.
// BALL: Always look at the ball.
// OBSTACLES: Look for obstacles.
enum class HeadFocus { NOTHING, LOC, BALL, BALL_SEARCH_LEFT, BALL_SEARCH_RIGHT, BALL_GOALIE, OBSTACLES, REFEREE };
