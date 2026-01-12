#pragma once

#include "bot.hpp"

namespace bot {

    // Display motor temperatures on controller screen
    void display_temperature();

    // State for middle scoring toggle
    extern bool mid_scoring;

    namespace intake_methods {
        void intake();
        void stop_intaking();
        void score_upper();
        void stop_scoring_upper();
        void outtake();
        void stop_outtaking();
        void score_middle();
        void stop_scoring_middle();
        void toggle_middle_score();
    }

    namespace buttons {
        void ButtonL1();
        void ButtonL1_released();
        void ButtonL2();
        void ButtonL2_released();
        void ButtonR1();
        void ButtonR1_released();
        void ButtonR2();
        void ButtonR2_released();
        void ButtonX();
        void ButtonY();
        void ButtonA();
        void ButtonB();
        void ButtonLeft();
        void ButtonRight();
        void ButtonDown();
        void ButtonUp();
    }
}
