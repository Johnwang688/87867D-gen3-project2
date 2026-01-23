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
            bot::pistons::arm_piston.set(true);
            bot::sensors::imu.set_heading(-90.0);
            bot::mcl::location.reset(400, -1500, 0);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(true);
            dt.drive_for(650, 1000, 100.0);
            dt.turn_to_heading(180, 1000, 100.0);
            dt.drive_for(375, 1000.0, 40.0);
            dt.match_load(3);
            dt.drive_for(-300, 1000, 100);
            bot::pistons::match_load_piston.set(false);
            dt.turn_to_heading(45, 1500, 100);
            bot::motors::lower.spin(vex::forward, 6.0, vex::voltageUnits::volt);
            dt.drive_for(450, 1500, 60);
            dt.turn_to_heading(2, 1500, 100);
            dt.drive_for(1300, 1500, 100);
            dt.brake();
            dt.turn_to_heading(90, 1000, 100);
            dt.drive_arc(180, -90, 1200, 50);
            dt.drive_for(-200, 500, 20);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive_for(30, 500, 30);
            vex::task adjust_heading = vex::task([]() -> int {
                dt.turn_to_heading(0, 1000, 100);
                task::sleep(1000);
                dt.turn_to_heading(0, 1000, 100);
                bot::motors::lower.spin(vex::forward, 100, vex::percent);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            vex::task::sleep(3000);
            bot::motors::intake.stop();
            adjust_heading.stop();
            bot::pistons::match_load_piston.set(true);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive_for(780, 1500, 40);
            adjust_heading = vex::task([]() -> int {
                dt.turn_to_heading(0, 1000, 100);
                return 0;
            });
            dt.match_load(3);
            adjust_heading.stop();
            dt.turn_to_heading(0, 1000, 50);
            dt.drive_for(-600, 1000, 100);
            dt.drive_for(-200, 500, 25);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            adjust_heading = vex::task([]() -> int {
                dt.turn_to_heading(0, 3000, 100);
                return 0;
            });
            vex::task::sleep(3000);
            bot::motors::intake.stop();
            adjust_heading.stop();
            dt.drive_for(300, 1000, 100);
            dt.turn_to_heading(-90, 1000, 100);
            bot::pistons::match_load_piston.set(false);
            dt.drive_for(2350, 4000, 100);
            bot::pistons::match_load_piston.set(true);
            dt.turn_to_heading(0, 1000, 100);
            dt.drive_for(350, 1000, 50);
            dt.match_load(3);
            dt.drive_for(-300, 1000, 100);
            bot::pistons::match_load_piston.set(false);
            dt.turn_to_heading(-135, 1000, 100);
            dt.drive_for(400, 1000, 100);
            dt.turn_to_heading(180, 1000, 100);
            dt.drive_for(1300, 1500, 100);
            dt.turn_to_heading(-90, 1000, 100);
            dt.drive_arc(180, -90, 1200, 50);
            dt.drive_for(-200, 500, 20);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive_for(30, 500, 30);
            adjust_heading = vex::task([]() -> int {
                dt.turn_to_heading(180, 1000, 100);
                task::sleep(1000);
                dt.turn_to_heading(180, 1000, 100);
                bot::motors::lower.spin(vex::forward, 100, vex::percent);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            vex::task::sleep(3000);
            bot::motors::intake.stop();
            adjust_heading.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive_for(780, 1500, 40);
            adjust_heading = vex::task([]() -> int {
                dt.turn_to_heading(180, 1000, 100);
                return 0;
            });
            dt.match_load(3);
            adjust_heading.stop();
            dt.turn_to_heading(180, 1000, 50);
            dt.drive_for(-600, 1000, 100);
            dt.drive_for(-200, 500, 25);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            adjust_heading = vex::task([]() -> int {
                dt.turn_to_heading(180, 3000, 100);
                bot::pistons::match_load_piston.set(false);
                return 0;
            });
            vex::task::sleep(3000);
            bot::motors::intake.stop();
            adjust_heading.stop();
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive_for(300, 1000, 100);
            dt.turn_to_heading(135, 1000, 100);
            dt.drive_for(500, 1000, 100);
            dt.turn_to_heading(-90, 1000, 100);
            bot::pistons::match_load_piston.set(true);
            while (bot::sensors::back_distance_right.objectDistance(mm) < 1600) {
                dt.tank_drive(100, 100);
            }
            dt.stop();
            dt.brake();
            bot::pistons::match_load_piston.set(false);
            return;
        }

        void left_long(){
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.turn_to_heading(180-13, 1000.0, 100.0);
            task piston_task = vex::task([]() -> int {
                task::sleep(450);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            dt.drive_for(650, 1000, 100);
            dt.turn_to_heading(180, 1000, 100);
            dt.drive_arc(-225, 180, 1500, 70);
            dt.drive_for(-300, 800, 30);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            vex::task adjust_heading = vex::task([]() -> int {
                dt.drive_for(-200, 500, 20);
                dt.turn_to_heading(0, 1000, 100);
                return 0;
            });
            vex::task::sleep(1000);
            bot::motors::intake.stop();
            adjust_heading.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive_for(750, 1200, 50);
            dt.match_load(1);
            dt.drive_for(-250, 1000, 35);
            dt.turn_to_heading(43, 1000, 100);
            dt.drive_for(-1050, 1500, 100);
            bot::motors::mid.spin(vex::reverse, 100, vex::percent);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            vex::task::sleep(1500);
            bot::motors::mid.stop();
            bot::motors::lower.stop();
            dt.drive_arc(300, 90, 1200, 100);
            dt.turn_to_heading(180-45, 500, 100);
            dt.drive_for(50, 500, 100);
            dt.drive_arc(300, 80, 1200, 100);
            return;

        }

        void left_4_mid_3_long(){
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::arm_piston.set(true);
            dt.turn_to_heading(180-11, 15000.0, 100.0);
            task piston_task = vex::task([]() -> int {
                task::sleep(800);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            dt.drive_for(655, 2000, 50);
            dt.turn_to_heading(45, 1000, 100);

            vex::task mid_scoring_task = vex::task([]() -> int {
                vex::task::sleep(400);
                bot::motors::mid.spin(vex::reverse, 60, vex::percent);
                return 0;
            });
            dt.drive_for(-400, 2000, 50);
            mid_scoring_task.stop();
            vex::task adjust_heading = vex::task([]() -> int {
                //dt.drive_for(-100, 500, 20);
                dt.turn_to_heading(45, 1000, 50);
                return 0;
            });
            vex::task::sleep(1000);
            adjust_heading.stop();
            bot::motors::mid.stop();
            dt.drive_for(1275, 3000, 50);
            dt.turn_to_heading(0, 1000, 100);
            dt.drive_for(450, 800, 40);
            //dt.match_load(1);
            /*dt.drive_for(-300, 1000, 50);
            dt.turn_to_heading(0, 1000, 100);
            dt.drive_for(-375, 1000, 50);*/
            dt.turn_to_heading(0, 500, 50);
            dt.drive_for(-650, 1000, 70);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            adjust_heading = vex::task([]() -> int {
                dt.turn_to_heading(0, 1500, 100);
                return 0;
            });
            bot::pistons::match_load_piston.set(false);
            vex::task::sleep(1500);
            //bot::pistons::arm_piston.set(false);
            dt.drive_arc(-100, -179, 1000, 40);
            bot::pistons::arm_piston.set(false);
            dt.turn_to_heading(180, 400, 100);
            dt.drive_for(200, 1000, 100);
            bot::motors::intake.stop();
            bot::motors::left_dt.setStopping(vex::brakeType::hold);
            bot::motors::right_dt.setStopping(vex::brakeType::hold);
            return;
            bot::pistons::match_load_piston.set(false);
            dt.drive_for(300, 1000, 100);
            dt.turn_to_heading(-45, 1000, 100);
            dt.drive_for(-400, 1000, 100);
            dt.turn_to_heading(0, 600, 100);
            dt.drive_for(-400, 650, 100);
            dt.stop();
            bot::motors::left_dt.setStopping(vex::brakeType::hold);
            bot::motors::right_dt.setStopping(vex::brakeType::hold);
            bot::Controller1.Screen.setCursor(1, 1);
            bot::Controller1.Screen.print(bot::Brain.Timer.time(vex::msec));
        }

        void left_7(){
            bot:motors::lower.spin(vex::forward, 100, vex::percent);
            dt.turn_to_heading(180-13, 1000, 100);
            vex::task piston_task = vex::task([]() -> int {
                task::sleep(450);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            
        }

        void right_7(){
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.turn_to_heading(180 + 13 - 360, 1000, 100);
            vex::task piston_task = vex::task([]() -> int {
                task::sleep(500);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            dt.drive_for(700, 1000, 100);
            dt.turn_to_heading(180, 1000, 100);
            dt.drive_arc(230, -180, 1500, 60);
            dt.drive_for(-300, 1000, 25);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive_for(25, 500, 20);
            vex::task adjust_heading = vex::task([]() -> int {
                dt.turn_to_heading(0, 1500, 100);
                return 0;
            });
            vex::task::sleep(1000);
            bot::motors::intake.stop();
            adjust_heading.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive_for(775, 1200, 30);
            dt.match_load(2);
            dt.turn_to_heading(2, 1000, 50);
            dt.drive_for(-600, 1000, 100);
            bot::motors::intake.spin(vex::forward, 100, percent);
            vex::task::sleep(1000);
            bot::motors::intake.stop();
            bot::pistons::match_load_piston.set(false);
            dt.drive_arc(150, 120, 1000, 30);
            dt.drive_for(150, 1000, 100);
            dt.turn_to_heading(15, 1000, 100);
            dt.drive_arc(400, -80, 1000, 50);
            dt.stop();
            bot::motors::left_dt.setStopping(vex::brakeType::hold);
            bot::motors::right_dt.setStopping(vex::brakeType::hold);
        }

        void left_9(){
            bot::pistons::arm_piston.set(true);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.turn_to_heading(180-11, 1000.0, 100.0);
            vex::task piston_task = vex::task([]() -> int {
                task::sleep(600);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            dt.drive_for(720.0, 1000.0, 100.0);
            bot::pistons::match_load_piston.set(false);
            dt.turn_to_heading(180-70, 1000.0, 100.0);
            vex::task piston = vex::task([]() -> int {
                task::sleep(400);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive_for(425, 1000.0, 100.0);
            dt.drive_for(-400, 1000, 100);
            dt.turn_to_heading(180, 1000, 100);
            //dt.drive_for(-400, 1000, 100);
            dt.drive_arc(-250, 190, 1500, 65);
            dt.drive_for(-200, 500, 30);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            vex::task adjust_heading = vex::task([]() -> int {
                //dt.drive_for(25, 500, 30);
                dt.turn_to_heading(0, 1000, 50);
                return 0;
            });
            vex::task::sleep(1000);
            bot::motors::intake.stop();
            adjust_heading.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive_for(850, 1300, 35);
            dt.match_load(1);
            dt.turn_to_heading(0, 500, 50);
            dt.drive_for(-650, 1000, 70);
            /*dt.drive_for(-300, 1000, 60);
            dt.turn_to_heading(0, 500, 100);
            dt.drive_for(-400, 1000, 50);*/
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            vex::task::sleep(1500);
            bot::motors::intake.stop();
            dt.drive_arc(-100, -179, 1000, 40);
            bot::pistons::arm_piston.set(false);
            dt.turn_to_heading(180, 400, 100);
            dt.drive_for(200, 1000, 100);
            bot::motors::left_dt.setStopping(vex::brakeType::hold);
            bot::motors::right_dt.setStopping(vex::brakeType::hold);
            return;


        }

        void left_4(){
            bot::pistons::arm_piston.set(true);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::motors::left_dt.setStopping(vex::brakeType::coast);
            bot::motors::right_dt.setStopping(vex::brakeType::coast);
            bot::pistons::match_load_piston.set(true);
            dt.drive_for(670, 1000, 100);
            dt.turn_to_heading(90, 1000, 100);
            dt.drive_for(350, 700, 40);
            dt.turn_to_heading(90, 500, 50);
            dt.drive_for(-650, 1000, 70);
            bot::pistons::match_load_piston.set(false);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            vex::task adjust_heading1 = vex::task([]() -> int {
                dt.turn_to_heading(90, 1000, 50);
                return 0;
            });
            vex::task::sleep(1500);
            bot::motors::intake.stop();
            adjust_heading1.stop();
            dt.drive_arc(-100, -179, 1000, 40);
            bot::pistons::arm_piston.set(false);
            dt.turn_to_heading(-90, 400, 100);
            dt.drive_for(250, 1000, 100);
            bot::motors::left_dt.setStopping(vex::brakeType::hold);
            bot::motors::right_dt.setStopping(vex::brakeType::hold);


            // middle blocks
            return;
            dt.turn_to_heading(180-11, 1000.0, 100.0);
            vex::task piston_task = vex::task([]() -> int {
                task::sleep(600);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            dt.drive_for(700, 1000, 100);
            dt.turn_to_heading(180, 1000, 100);
            dt.drive_arc(-225, 180, 1500, 100);
            dt.drive_for(-200, 500, 30);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            vex::task adjust_heading = vex::task([]() -> int {
                //dt.drive_for(25, 500, 30);
                dt.turn_to_heading(0, 500, 50);
                return 0;
            });
            bot::pistons::match_load_piston.set(false);
            vex::task::sleep(1000);
            dt.drive_arc(-100, -179, 1000, 40);
            bot::pistons::arm_piston.set(false);
            dt.turn_to_heading(180, 400, 100);
            dt.drive_for(250, 1000, 100);
            bot::motors::left_dt.setStopping(vex::brakeType::hold);
            bot::motors::right_dt.setStopping(vex::brakeType::hold);
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1, 1);
            bot::Controller1.Screen.print("time: %f", bot::Brain.Timer.time(vex::msec));
            while(1) {
                task::sleep(100);
            }
            
        }

        void right_long(){
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.turn_to_heading(180 + 13 - 360, 1000, 100);
            task piston_task = vex::task([]() -> int {
                task::sleep(400);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            dt.drive_for(600, 1000, 100);
            dt.turn_to_heading(180, 1000, 100);
            dt.drive_arc(210, -180, 1500, 65);
            dt.drive_for(-300, 1000, 25);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive_for(35, 500, 15);
            vex::task adjust_heading = vex::task([]() -> int {
                dt.turn_to_heading(0, 1500, 100);
                return 0;
            });
            vex::task::sleep(1500);
            bot::motors::intake.stop();
            adjust_heading.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive_for(800, 1200, 50);
            dt.match_load(1);
            dt.drive_for(-200, 1000, 100);
            dt.turn_to_heading(-45, 1000, 100);
            dt.drive_for(-375, 2000, 100);
            bot::motors::lower.stop();
            dt.turn_to_heading(0, 1500, 100);
            dt.drive_for(-550, 2000, 60);
        }

        void sawp(){
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::arm_piston.set(true);
            dt.drive_for(100, 500, 100);
            vex::task piston_task = vex::task([]() -> int {
                task::sleep(1200);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            dt.drive_for(-1180, 1500, 100, 0);
            dt.turn_to_heading(-90, 1000, 100, true);
            dt.drive_for(500, 700, 50, -90);
            dt.match_load(1);
            dt.drive_for(-700, 1000, 100, -90);
            bot::pistons::match_load_piston.set(false);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            vex::task adjust_heading = vex::task([]() -> int {
                dt.drive_for(-200, 800, 0);
                return 0;
            });
            vex::task::sleep(800);
            bot::motors::intake.stop();
            adjust_heading.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            piston_task = vex::task([]() -> int {
                vex::task::sleep(1300);
                bot::pistons::match_load_piston.set(true);
                vex::task::sleep(200);
                bot::pistons::match_load_piston.set(false);
                vex::task::sleep(600);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            dt.drive_arc(85, 135, 1000, 100);
            dt.drive_for(1450, 1500, 100, 0);
            bot::pistons::match_load_piston.set(true);
            vex::task mid_scoring_task = vex::task([]() -> int {
                vex::task::sleep(500);
                bot::motors::mid.spin(vex::reverse, 50, vex::percent);
                return 0;
            });
            dt.drive_for(-450, 1000, 50, -45);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::motors::upper.spin(vex::forward, 100, vex::percent);
            vex::task::sleep(800);
            bot::motors::mid.stop();
            bot::motors::upper.stop();
            dt.drive_for(1120, 1500, 100, -45);
            dt.drive_for(500, 1000, 50, -90);
            dt.match_load(1);
            /*dt.turn_to_heading(-90, 1000, 100, true);
            dt.drive_for(350, 1000, 50, -90);*/
            dt.drive_for(-700, 1000, 100, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);

        }

        void test(){
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            vex::task piston_task = vex::task([]() -> int {
                task::sleep(700);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            dt.drive_for(650, 1000, 100, 15);
            dt.drive_for(-800, 1500, 60, -70);
            dt.brake();
            dt.turn_to_heading(180, 1000, 100, true);
            dt.drive_for(580, 1000, 50, 180);
            dt.turn_to_heading(180, 500, 30, true);
            dt.drive_for(-700, 1000, 100, -176);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            vex::task back_up = vex::task([]() -> int {
                dt.turn_to_heading(180, 1500, 50, true);
                return 0;
            });
            vex::task::sleep(2000);
            bot::motors::intake.stop();
            dt.drive_for(250, 1000, 100, -100);
            dt.coast();
            dt.drive_for(-200, 1000, 60, 180);
            dt.drive_for(-350, 1000, 100);
            dt.brake();

            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1, 1);
            bot::Controller1.Screen.print("heading: %f", bot::sensors::left_imu.heading(vex::degrees));
            bot::Controller1.Screen.setCursor(2, 1);
            bot::Controller1.Screen.print("time: %f", bot::Brain.Timer.time(vex::msec));
            dt.stop();
            dt.brake();
            return;
        }
    }
}