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
        void sawp();
        void skills();
        void test();
    }

    namespace drivetrains {
        extern Drivetrain dt;
    }

    namespace mcl {
        extern Location location;
    }
}

