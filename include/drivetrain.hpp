#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "pid.hpp"
#include "location.hpp"
#include "bot.hpp"

#include <vector>
#include <cstdint>
#include <cmath>

using namespace vex;

class Drivetrain {
    public:
        Drivetrain(motor_group& left_motor, motor_group& right_motor,
            bot::sensors::inertial_group& imu);
        
        void drive_for(double distance, double timeout, double speed_limit);
        void turn_to_heading(double heading, double timeout, double speed_limit);
        void tank_drive(double left_speed, double right_speed);
        void arcade_drive(double fwd, double turn);
        void match_load(std::int8_t times);
        void follow_path(std::vector<Waypoint>& path, Location& location, 
            double lookahead, double maxSpeed, double stopDist = 15.0);
        void drive_arc(double radius, double angle, double timeout, double speed_limit);

        double get_left_encoder();
        double get_right_encoder();

    private:
        motor_group& _left_motor;
        motor_group& _right_motor;
        double _wheel_diameter;
        double _track_width;
        double _gear_ratio;
        double _drive_kp;
        double _drive_ki;
        double _drive_kd;
        double _turn_kp;
        double _turn_ki;
        double _turn_kd;
        double _max_voltage;
        double _max_accel;
        bot::sensors::inertial_group& _imu;
        PID _left_drive_pid;
        PID _right_drive_pid;
        PID _left_turn_pid;
        PID _right_turn_pid;
        PID _left_arc_pid;
        PID _right_arc_pid;
        
        // Pure Pursuit helpers
        struct IntersectionPoint { double x, y, t; };
        std::vector<IntersectionPoint> circleSegmentIntersections(
            double cx, double cy, double radius,
            double x1, double y1, double x2, double y2);
        void handle_intake_mode(IntakeMode mode);
};

#endif // DRIVETRAIN_HPP