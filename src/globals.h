#pragma once

static const float BALL_MASS = 5.0f;
static const float BALL_RADIUS = 0.25;
static const float PLAYER_MASS = 20.0;
static const float PLAYER_HEIGHT = 1.8;
static const float PLAYER_RADIUS = 0.3;
static const float PLAYER_JUMP_VELOCITY = 8.0;
static const float PLAYER_WALK_SPEED = 5.0;
static const float PLAYER_WALK_ACCEL = 200.0;

namespace PhysicsUserIndex { // TODO: use enum class while allowing conversion to int

enum Enum
{
    None,
    Player,
    JumpPad,
    Ladder,
    Elevator
};

} // namespace PhysicsUserIndex
