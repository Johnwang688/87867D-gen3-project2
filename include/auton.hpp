#pragma once

#include "bot.hpp"
#include "drivetrain.hpp"
#include "location.hpp"

namespace bot {
    namespace auton {
        void left_long_9();
        void left_two_goal();
        void right_long_9();
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

