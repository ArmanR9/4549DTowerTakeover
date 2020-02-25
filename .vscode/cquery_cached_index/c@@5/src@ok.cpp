#include "ok.hpp"
#include "okapi/api.hpp"
using namespace okapi::literals;
using namespace okapi;
/*
std::shared_ptr<OdomChassisController> chassis =
ChassisControllerBuilder()
    .withMotors( // left motor is 1, right motor is 2 (reversed)
    {-5, -8}, // Left motors are 1 & 2 (reversed)
       {6, 7})    // Right motors are 3 & 4
    .withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 12.58_in}, okapi::imev5GreenTPR})
    .withSensors(
      ADIEncoder {'A','B'},
      ADIEncoder{'E','F', true},
      ADIEncoder {'C','D', true}

    )
    // specify the tracking wheels diameter (3 in), track (7 in), and TPR (360)
    // specify the middle encoder distance (1 in) and diameter (2.75 in)
    .withOdometry({{2.769011_in, 4.725_in, 2_in, 2.769011_in}, okapi::quadEncoderTPR})
    .buildOdometry();
*/
