#pragma once
#include "types.hpp"

// PID constants
constexpr double DRIVE_KP = 0.09;
constexpr double DRIVE_KI = 0.01;
constexpr double DRIVE_KD = 0.001;

constexpr double TURN_KP = 0.09;
constexpr double TURN_KI = 0.01;
constexpr double TURN_KD = 0.001;

constexpr double ARC_KP = 0.08;
constexpr double ARC_KI = 0.01;
constexpr double ARC_KD = 0.005;

constexpr double MAX_INTEGRAL = 1000.0;
constexpr double MAX_OUTPUT = 100.0;
constexpr double DT = 0.02; // seconds 

//mm
constexpr double WHEEL_DIAMETER = 88.25;
constexpr double TRACK_WIDTH = 330.0;
constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

// Gear ratio is output speed / input speed
constexpr double GEAR_RATIO = 400.0/600.0;
constexpr double MM_PER_TICK = (WHEEL_CIRCUMFERENCE / 360.0) * GEAR_RATIO;

// Maximum voltage to apply to motors
constexpr double MAX_VOLTAGE = 11.0;

// controller deadzone 
constexpr double CONTROLLER_DEADZONE = 1.0;
constexpr double MID_SPEED = 100.0;

// max accel gain for PID controllers 
constexpr double MAX_ACCEL = 3.5;

// driver sens constants
constexpr double A = 2.0;
constexpr double B = 1.04;
constexpr double C = -A;

// Particle filter constants
constexpr std::uint8_t LAYERS = 6;
constexpr std::uint16_t INITIAL_STEP_SIZE = 800;
constexpr std::uint16_t MAX_PARTICLES = 22 * (LAYERS-1);

// Robot dimensions and offsets (in mm)
constexpr std::int16_t LEFT_FORWARD_OFFSET_X = 0;
constexpr std::int16_t LEFT_FORWARD_OFFSET_Y = 0;
constexpr std::int16_t RIGHT_FORWARD_OFFSET_X = 0;
constexpr std::int16_t RIGHT_FORWARD_OFFSET_Y = 0;

constexpr std::int16_t LEFT_AFT_OFFSET_X = 0;
constexpr std::int16_t LEFT_AFT_OFFSET_Y = 0;
constexpr std::int16_t RIGHT_AFT_OFFSET_X = 0;
constexpr std::int16_t RIGHT_AFT_OFFSET_Y = 0;

constexpr std::int16_t FRONT_LEFT_OFFSET_X = 0;
constexpr std::int16_t FRONT_LEFT_OFFSET_Y = 0;
constexpr std::int16_t FRONT_RIGHT_OFFSET_X = 0;
constexpr std::int16_t FRONT_RIGHT_OFFSET_Y = 0;

constexpr std::int16_t BACK_LEFT_OFFSET_X = 0;
constexpr std::int16_t BACK_LEFT_OFFSET_Y = 0;
constexpr std::int16_t BACK_RIGHT_OFFSET_X = 0;
constexpr std::int16_t BACK_RIGHT_OFFSET_Y = 0;

// field dimensions (in mm)
constexpr std::int16_t WIDTH = 3658;
constexpr std::int16_t HEIGHT = 3658;
constexpr std::int16_t OUT_OF_BOUNDS = WIDTH/2;

// All units in mm. Origin (0,0) is center.
constexpr Line map[] = {
    // 1. Perimeter Walls
    {-WIDTH/2, -HEIGHT/2,  WIDTH/2, -HEIGHT/2}, { WIDTH/2, -HEIGHT/2,  WIDTH/2,  HEIGHT/2},
    { WIDTH/2,  HEIGHT/2, -WIDTH/2,  HEIGHT/2}, {-WIDTH/2,  HEIGHT/2, -WIDTH/2, -HEIGHT/2},

    // 2. West Long Goal "V" Legs
    {-1500, -300, -HEIGHT/2,    0}, // Leg 1
    {-1500,  300, -HEIGHT/2,    0}, // Leg 2

    // 3. East Long Goal "V" Legs
    { 1500, -300,  HEIGHT/2,    0}, // Leg 1
    { 1500,  300,  HEIGHT/2,    0}, // Leg 2

    // 4. Center Goal Vertical Supports (Parallel Lines)
    {-150,  300,  150,  300}, // Center Goal North Support
    {-150, -300,  150, -300}  // Center Goal South Support
};