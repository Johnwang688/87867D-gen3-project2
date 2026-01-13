#pragma once

#include "constants.hpp"
#include "robot_config.hpp"
#include <algorithm>
#include <cmath>

namespace helpers {

    inline double mmToDegrees(double mm) {
        return mm / MM_PER_TICK;
    }

    inline double degreesToDegrees(double degrees) {
        return degrees * (TRACK_WIDTH / WHEEL_DIAMETER);
    }

    inline double wrapToPi(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Wraps angle to -180 to 180 range (degrees)
    inline double wrap_to_180(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }

    // positive is counter clockwise. 
    // return range -180 to 180
    inline double angular_difference(double from, double to) {
        double diff = to - from;
        return wrap_to_180(diff);
    }

    inline double distancePointToSegment(double px, double py, double x1, double y1, double x2, double y2){
        double vx = x2 - x1;
        double vy = y2 - y1;
        double wx = px - x1;
        double wy = py - y1;
        double c1 = vx * wx + vy * wy;
        if (c1 <= 0) return std::hypot(px - x1, py - y1);
        double c2 = vx * vx + vy * vy;
        if (c2 <= c1) return std::hypot(px - x2, py - y2);
        double t = c1 / c2;
        double projx = x1 + t * vx;
        double projy = y1 + t * vy;
        return std::hypot(px - projx, py - projy);
    }   

    inline double average_heading(double h1, double h2) {
        double diff = h2 - h1;
        if (diff > 180.0) diff -= 360.0;
        else if (diff < -180.0) diff += 360.0;
        double avg = h1 + diff / 2.0;
        // Normalize to -180 to 180
        if (avg > 180.0) avg -= 360.0;
        else if (avg <= -180.0) avg += 360.0;
        return avg;
    }

}

namespace math {
    inline double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, value));
    }

    inline double to_rad(double angle) {
        return angle * M_PI / 180.0;
    }

    inline double curve(double input){
        return A * std::pow(B, input) + C;
    }

    inline double robot_to_math_angle(double robot_angle_deg) {
        return 90.0 - robot_angle_deg;
    }
    
    // Get direction vector in world coordinates from robot frame angle
    // Returns (dx, dy) where dx is world X component, dy is world Y component
    inline void robot_angle_to_direction(double robot_angle_deg, float& dx, float& dy) {
        double rad = to_rad(robot_angle_deg);
        //in robot frame: 0° = +Y; forward = (0, 1)
        //sin(0°) = 0, cos(0°) = 1, so (sin, cos) gives us forward direction
        dx = static_cast<float>(std::sin(rad));  // Right component
        dy = static_cast<float>(std::cos(rad));  // Forward component
    }
    
}