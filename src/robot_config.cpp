#include "robot_config.hpp"

namespace bot {
	vex::brain Brain;
	vex::competition Competition;
	vex::controller Controller1 = vex::controller(vex::primary);

	namespace motors {
		vex::motor leftA = vex::motor(vex::PORT11, vex::ratio6_1, true);
		vex::motor leftB = vex::motor(vex::PORT19, vex::ratio6_1, false);
		vex::motor leftC = vex::motor(vex::PORT20, vex::ratio6_1, true);

		vex::motor rightA = vex::motor(vex::PORT1, vex::ratio6_1, false);
		vex::motor rightB = vex::motor(vex::PORT9, vex::ratio6_1, true);
		vex::motor rightC = vex::motor(vex::PORT10, vex::ratio6_1, false);

		vex::motor_group left_dt = vex::motor_group(leftA, leftB, leftC);
		vex::motor_group right_dt = vex::motor_group(rightA, rightB, rightC);
		vex::motor_group all = vex::motor_group(leftA, leftB, leftC, rightA, rightB, rightC);

		vex::motor upper = vex::motor(vex::PORT2, vex::ratio18_1, true);
		vex::motor mid = vex::motor(vex::PORT17, vex::ratio18_1, false);
		vex::motor lower = vex::motor(vex::PORT3, vex::ratio6_1, false);
		vex::motor_group intake = vex::motor_group(upper, mid, lower);
	}

	namespace sensors {
		vex::inertial left_imu = vex::inertial(vex::PORT18);
		vex::inertial right_imu = vex::inertial(vex::PORT5);
		bot::sensors::inertial_group imu = bot::sensors::inertial_group(left_imu, right_imu);

		vex::distance left_distance_forward = vex::distance(vex::PORT12);
		vex::distance left_distance_aft = vex::distance(vex::PORT13);
		vex::distance right_distance_forward = vex::distance(vex::PORT14);
		vex::distance right_distance_aft = vex::distance(vex::PORT15);
		vex::distance front_distance_left = vex::distance(vex::PORT16);
		vex::distance front_distance_right = vex::distance(vex::PORT17);
		vex::distance back_distance_left = vex::distance(vex::PORT18);
		vex::distance back_distance_right = vex::distance(vex::PORT19);
	}

	namespace pistons {
		vex::digital_out match_load_piston = vex::digital_out(bot::Brain.ThreeWirePort.C);
		vex::digital_out arm_piston = vex::digital_out(bot::Brain.ThreeWirePort.F);
	}

}