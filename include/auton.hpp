#pragma once

#include "bot.hpp"
#include "drivetrain.hpp"
#include "location.hpp"

namespace bot {
    namespace auton {
        void left_long();
        void left_9();
        void left_7();
        void right_long();
        void right_7();
        void left_4_mid_3_long();
        void left_4();
        void sawp();
        void skills();
        void test();
        void skills_2();
    }

    namespace drivetrains {
        extern Drivetrain dt;
    }

    namespace mcl {
        extern Location location;
    }
}

