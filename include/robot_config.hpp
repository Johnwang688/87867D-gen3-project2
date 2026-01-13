#pragma once

#include "vex.h"

namespace bot {
    extern vex::brain Brain;
    extern vex::competition Competition;
    extern vex::controller Controller1;

    namespace motors {
        extern vex::motor leftA;
        extern vex::motor leftB;
        extern vex::motor leftC;

        extern vex::motor_group left_dt;

        extern vex::motor rightA;
        extern vex::motor rightB;
        extern vex::motor rightC;

        extern vex::motor_group right_dt;

        extern vex::motor_group all;

        extern vex::motor upper;
        extern vex::motor mid;
        extern vex::motor lower;

        extern vex::motor_group intake;

    }

    namespace sensors {
        extern vex::inertial left_imu;
        extern vex::inertial right_imu;

        class inertial_group {
            public:
                inertial_group(vex::inertial& imu_left, vex::inertial& imu_right)
                    : _imu_left(imu_left), _imu_right(imu_right) {}
                void calibrate() {
                    _imu_left.calibrate();
                    _imu_right.calibrate();
                    while (_imu_left.isCalibrating() || _imu_right.isCalibrating()) {
                        vex::task::sleep(10);
                    }
                }
                double get_heading() {
                    double right_rad = _imu_right.heading(vex::degrees) * M_PI / 180.0;
                    double left_rad = _imu_left.heading(vex::degrees) * M_PI / 180.0;
                    
                    // Average the unit vectors (circular mean)
                    double avg_x = (cos(right_rad) + cos(left_rad)) / 2.0;
                    double avg_y = (sin(right_rad) + sin(left_rad)) / 2.0;
                
                    double avg_heading = -atan2(avg_y, avg_x) * 180.0 / M_PI;
                    
                    avg_heading += 90.0;
                    
                    // Normalize to -180 to 180
                    if (avg_heading > 180.0) avg_heading -= 360.0;
                    if (avg_heading <= -180.0) avg_heading += 360.0;
                    
                    return avg_heading;
                }

            private:
                vex::inertial& _imu_left;
                vex::inertial& _imu_right;
        };

        extern inertial_group imu;

        extern vex::distance left_distance_forward;
        extern vex::distance left_distance_aft; 
        extern vex::distance right_distance_forward; 
        extern vex::distance right_distance_aft; 
        extern vex::distance front_distance_left; 
        extern vex::distance front_distance_right; 
        extern vex::distance back_distance_left; 
        extern vex::distance back_distance_right;
    }

    namespace pistons {
        extern vex::digital_out match_load_piston;
        extern vex::digital_out arm_piston;

        inline void toggle_match_load_piston(){
            bot::pistons::match_load_piston.set(!bot::pistons::match_load_piston.value());
        }

        inline void toggle_arm_piston(){
            bot::pistons::arm_piston.set(!bot::pistons::arm_piston.value());
        }
    }
}