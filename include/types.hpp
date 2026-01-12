#pragma once

#include <cstdint>
#include <vector>

struct Line {
    float x1, y1, x2, y2;
};

struct Particle {
    std::int16_t x, y, heading;
    std::uint16_t front_left, front_right, back_left, back_right, 
    left_forward, left_aft, right_forward, right_aft;
    bool valid;  // Flag to indicate if particle is within bounds
};

struct Reading {
    std::uint16_t front_left, front_right, back_left, back_right,
    left_forward, left_aft, right_forward, right_aft;
};

enum class IntakeMode : std::int8_t {
    STOP = 0,
    INTAKE = 1,
    OUTTAKE = -1,
};

struct Waypoint {
    double x;
    double y;
    IntakeMode intake_mode;
};
