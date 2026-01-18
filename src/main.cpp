/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       johnw                                                     */
/*    Created:      1/11/2026, 2:11:49 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

#include "bot.hpp"
#include "auton.hpp"
#include "buttons.hpp"

using namespace vex;
using namespace bot;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
    bot::sensors::left_imu.calibrate();
    bot::sensors::right_imu.calibrate();
    while (bot::sensors::left_imu.isCalibrating() || bot::sensors::right_imu.isCalibrating()) {
        vex::task::sleep(10);
    }
    //bot::mcl::location.reset(400, 1200, 90.0);
    //bot::mcl::location.start();
    //printf("IMU: %.2f\n", bot::sensors::imu.get_heading());
    //printf("Location: %d, %d, %d\n", bot::mcl::location.get_x(), bot::mcl::location.get_y(), bot::mcl::location.get_heading());
    return;
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  bot::auton::left_9();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

  bot::drivetrains::dt.coast();

  //bot::sensors::imu.calibrate();
  //vex::task::sleep(500);
  
  // Debug: Print IMU heading after calibration
  //printf("IMU heading after calibration: %.2f\n", bot::sensors::imu.get_heading());
  
  //bot::mcl::location.reset(300, -1200, 0);
  //bot::mcl::location.start();

  bot::pistons::toggle_arm_piston();

  bot::Controller1.ButtonL1.pressed(bot::buttons::ButtonL1);
  bot::Controller1.ButtonL1.released(bot::buttons::ButtonL1_released);
  bot::Controller1.ButtonL2.pressed(bot::buttons::ButtonL2);
  bot::Controller1.ButtonL2.released(bot::buttons::ButtonL2_released);
  bot::Controller1.ButtonR1.pressed(bot::buttons::ButtonR1);
  bot::Controller1.ButtonR1.released(bot::buttons::ButtonR1_released);
  bot::Controller1.ButtonR2.pressed(bot::buttons::ButtonR2);
  bot::Controller1.ButtonR2.released(bot::buttons::ButtonR2_released);

  bot::Controller1.ButtonA.pressed(bot::buttons::ButtonA);
  bot::Controller1.ButtonB.pressed(bot::buttons::ButtonB);
  bot::Controller1.ButtonX.pressed(bot::buttons::ButtonX);
  bot::Controller1.ButtonY.pressed(bot::buttons::ButtonY);
  bot::Controller1.ButtonY.released(bot::buttons::ButtonY_released);

  bot::Controller1.ButtonLeft.pressed(bot::buttons::ButtonLeft);
  bot::Controller1.ButtonRight.pressed(bot::buttons::ButtonRight);
  bot::Controller1.ButtonDown.pressed(bot::buttons::ButtonDown);
  bot::Controller1.ButtonUp.pressed(bot::buttons::ButtonUp);
  
  // variables for driver control
  double leftY, leftX, rightY, rightX;
  double left_joystick, right_joystick;
  double left, right;

  while (1) {
    leftY = bot::Controller1.Axis3.position();
    leftX = bot::Controller1.Axis4.position();
    rightY = bot::Controller1.Axis2.position();
    rightX = bot::Controller1.Axis1.position();

    // calculate joystick magnitudes
    left_joystick = sqrt(leftY * leftY + leftX * leftX);
    right_joystick = sqrt(rightY * rightY + rightX * rightX);


    //left_joystick = math::curve(left_joystick);
    //right_joystick = math::curve(right_joystick);

    // deadzone adjustment
    if (fabs(left_joystick) < CONTROLLER_DEADZONE) left_joystick = 0.0;
    if (fabs(right_joystick) < CONTROLLER_DEADZONE) right_joystick = 0.0;
     
    left = (leftY < 0) ? -left_joystick : left_joystick;
    right = (rightY < 0) ? -right_joystick : right_joystick;

    left = math::clamp(left, -100, 100);
    right = math::clamp(right, -100, 100);

    bot::drivetrains::dt.tank_drive(left, right);

    vex::task::sleep(20); // Sleep the task for a short amount of time to
                          // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  bot::Competition.autonomous(autonomous);
  bot::Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
