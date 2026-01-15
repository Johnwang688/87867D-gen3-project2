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
            std::vector<Waypoint> path1 = {
                {-450, 600, IntakeMode::INTAKE},
                {-600, 600, IntakeMode::INTAKE},
                {-1100, 400, IntakeMode::INTAKE},
                //{-700, 500, IntakeMode::INTAKE},
                //{-1200, 1200, IntakeMode::INTAKE},
                /*
                {1000, 0, IntakeMode::INTAKE},
                {1000, 1000, IntakeMode::INTAKE},
                {0, 1000, IntakeMode::INTAKE},
                {0, 0, IntakeMode::INTAKE}*/
            };
            task::sleep(2000);
            /*dt.turn_to_heading(90, 3000, 40);
            return;
            dt.drive_to(-450, 600, 1200, 40);
            dt.drive_to(-1000, 200, 1500, 40);
            return;
            */
            dt.follow_path(path1, bot::mcl::location, 1000, 100, 150);
            dt.brake();
            dt.stop();
        }
    }
}