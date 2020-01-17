#include "main.h"
#include "gui.hpp"
#include "auto_drive.hpp"
#include "okapi/api.hpp"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */




using namespace okapi::literals;
using namespace okapi;

  /*  std::shared_ptr<okapi::AsyncMotionProfileController> profileController = okapi::AsyncMotionProfileControllerBuilder()
                           .withLimits({1.0, 2.0, 10.0})
                           .withOutput(chassis_)
                           .buildMotionProfileController();
                           */




void autonomous() {

  std::shared_ptr<okapi::OdomChassisController> chassis_ =
    okapi::ChassisControllerBuilder()
      .withMotors({5, -6}, // Left motors are 1 & 2 (reversed)
                  {8, -7})    // Right motors are 3 & 4 // left motor is 1, right motor is 2 (reversed)
      // green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
      .withDimensions(okapi::AbstractMotor::gearset::green, {{4.125_in, 10.5_in}, okapi::imev5GreenTPR})
      // left encoder in ADI ports A & B, right encoder in ADI ports C & D (reversed)
      .withSensors(okapi::ADIEncoder{'A','B'}, okapi::ADIEncoder{'E','F', true}, okapi::ADIEncoder{'C','D', true} )
      // specify the tracking wheels diameter (3 in), track (7 in), and TPR (360)
      .withOdometry({{2.75_in, 4.725_in, 2_in, 2.75_in}, quadEncoderTPR}, StateMode::CARTESIAN)
      .buildOdometry();

  //profileController->generatePath(
  //  {{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}, "A");
  //profileController->setTarget("A");
//  profileController->waitUntilSettled();
//  */

  switch(auton_sel){

  case(1):
  drive_point(0, 15, 0, 0, 0, 50, 500, 2500);
  pros::delay(10000);
  break;

  case(2):
  //blueAuto();
  break;

  default:
  chassis_->setState({0_in, 0_in, 0_deg});
  chassis_->driveToPoint({10_in, 10_in});
  //  drive_point(0, 15, 0, 0, 0, 50, 500, 2500);
  //drive_point(5.0, 0.0, 0.0, 0.0, 0.0, 25.0, 1000, 2500);
  break;
  }

}
