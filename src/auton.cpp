#include "auton.hpp"

namespace bot {
    namespace drivetrains {
        Drivetrain dt = Drivetrain(bot::motors::left_dt, bot::motors::right_dt, bot::sensors::imu);
    }
    namespace mcl {
        Location location = Location(bot::drivetrains::dt, 
            bot::sensors::left_distance_forward, 
            bot::sensors::right_distance_forward, 
            bot::sensors::left_distance_aft, 
            bot::sensors::right_distance_aft, 
            bot::sensors::front_distance_left, 
            bot::sensors::front_distance_right, 
            bot::sensors::back_distance_left, 
            bot::sensors::back_distance_right, 
            bot::sensors::imu);
    }

    namespace auton {

        using namespace bot::drivetrains;

        void skills(){

        }

        void left_long_9(){

        }

        void left_two_goal(){

        }

        void right_long_9(){

        }

        void sawp(){

        }

        void test(){

        }
    }
}