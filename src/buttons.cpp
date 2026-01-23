#include "buttons.hpp"
#include "location_debug.hpp"
#include <vector>

namespace bot {

    void display_temperature() {
        double max_left_temp = 0.0, max_right_temp = 0.0;
        // list of left drivetrain motors
        std::vector<double> left_motors = {
          bot::motors::leftA.temperature(),
          bot::motors::leftB.temperature(),
          bot::motors::leftC.temperature()
        };
        // find which motor has the highest temperature
        for (int i = 0; i < left_motors.size(); i++) {
          max_left_temp = std::fmax(max_left_temp, left_motors[i]);
        }
        
        // list of right drivetrain motors
        std::vector<double> right_motors = {
          bot::motors::rightA.temperature(),
          bot::motors::rightB.temperature(), 
          bot::motors::rightC.temperature()
        };
        // find which motor has the highest temperature
        for (int i = 0; i < right_motors.size(); i++) {
          max_right_temp = std::fmax(max_right_temp, right_motors[i]);
        }
        
        //intake motor temperatures
        double lower_temp = bot::motors::lower.temperature();
        double upper_temp = bot::motors::upper.temperature();
        // display temperatures on the brain screen
        bot::Controller1.Screen.clearScreen();
        bot::Controller1.Screen.setCursor(1,1);
        bot::Controller1.Screen.print("Left Max: %.1f%%", max_left_temp);
        bot::Controller1.Screen.setCursor(2,1);
        bot::Controller1.Screen.print("Right Max: %.1f%%", max_right_temp);
        bot::Controller1.Screen.setCursor(3,1);
        bot::Controller1.Screen.print("L: %.1f%%", lower_temp);
        bot::Controller1.Screen.setCursor(3, 15);
        bot::Controller1.Screen.print("U: %.1f%%", upper_temp);
    }

    bool mid_scoring = false;

    namespace intake_methods {
        using namespace bot::motors;
        using namespace vex;
        void intake(){
            lower.spin(forward, 100, percent);
        }
        void stop_intaking(){
            lower.stop();
        }
        void score_upper(){
            upper.spin(forward, 100, percent);
            mid.spin(forward, 100, percent);
        }
        void stop_scoring_upper(){
            upper.stop();
            mid.stop();
        }
        void outtake(){
            lower.spin(reverse, 100, percent);
        }
        void stop_outtaking(){
            lower.stop();
        }

        void score_middle(){
            bot::mid_scoring = true;
            mid.spin(reverse, MID_SPEED, percent); 
        }

        void stop_scoring_middle(){
            bot::mid_scoring = false;
            mid.stop();
        }

        void toggle_middle_score(){
            if (bot::mid_scoring){
                bot::intake_methods::stop_scoring_middle();
            } else {
                bot::intake_methods::score_middle();
            }
        }
    }

    namespace buttons {

        using namespace bot::intake_methods;
        using namespace bot::pistons;

        void ButtonL1(){
            bot::pistons::arm_piston.set(false);
            //bot::motors::upper.spin(vex::forward, 100, vex::percent);
            //bot::motors::mid.spin(vex::forward, 50, vex::percent);
        }
        void ButtonL1_released(){
            bot::pistons::arm_piston.set(true);
            //bot::motors::upper.stop();
            //bot::motors::mid.stop();
        }
        void ButtonL2(){
            bot::intake_methods::score_upper();
            //bot::motors::upper.spin(vex::reverse, 100, vex::percent);
            //bot::motors::mid.spin(vex::reverse, 50, vex::percent);
        }
        void ButtonL2_released(){
            bot::intake_methods::stop_scoring_upper();
            //bot::motors::upper.stop();
            //bot::motors::mid.stop();
        }
        void ButtonR1(){
            bot::intake_methods::intake();
        }
        void ButtonR1_released(){
            bot::intake_methods::stop_intaking();
        }
        void ButtonR2(){
            bot::intake_methods::outtake();
        }
        void ButtonR2_released(){
            bot::intake_methods::stop_outtaking();
        }
        void ButtonX(){
            bot::display_temperature();
        }
        void ButtonY(){
            bot::intake_methods::toggle_middle_score();
            //bot::intake_methods::score_middle();
            //bot::intake_methods::intake();
        }
        void ButtonY_released(){
            //bot::intake_methods::stop_scoring_middle();
            //bot::intake_methods::stop_intaking();
        }
        void ButtonA(){
            //debug::print_sensor_data();
        }
        void ButtonB(){
            bot::pistons::toggle_match_load_piston();
        }
        void ButtonLeft(){
            //debug::print_location();
        }
        void ButtonRight(){

        }
        void ButtonDown(){
            //bot::pistons::toggle_arm_piston();
        }
        void ButtonUp(){
            
        }
    }
}