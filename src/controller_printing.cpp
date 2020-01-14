#include "controller_printing.hpp"
#include "main.h"
#include <string>
#include <array>
#include "sensors.hpp"
#include "motors.hpp"
#include "joystick.hpp"
#include "odometry.hpp"

using namespace pros;

std::array<std::string, 3> lines = {}; // init string arrays
std::array<std::string, 3> lastLines = {};


void controller_print(void* param){

DriverProfile* profile = (DriverProfile*)param; //cast void pointer param into type Drive Profile
// create profile ptr to param, and use it access the current members of the passed in object

 lines[0] = "init";
 lines[1] = "init";
 lines[2] = "init";

  const int maxWidth = 15;

  while(true) {

    bool delayed = false;

    switch(profile->getDrive()){            // Switch case for switching printed strings on controller (Driver Layout)
      case (DriverProfile::Drive_State::TANK_DRIVE):
          lines[0] = "Tank";
        break;

        case (DriverProfile::Drive_State::ARCADE_DRIVE):
            lines[0] = "Arcade";
          break;

          default:
            lines[0] = "NULL";
          break;
    }

     switch(profile->getCurve()){ // Switch case for switching printed strings on controller (Curve setting)
     case (DriverProfile::Curve_State::CUBIC_DRIVE):
     lines[1] = "Cubic Curve";
     break;

     case(DriverProfile::Curve_State::QUADRATIC_DRIVE):
     lines[1] = "Quad curve";
     break;


     default:
     lines[1] = "Default curve";
     break;
   }

   switch(profile->get_sModifier()){ // Switch case for switching printed strings on controller (Joystick Modifier setting)

    case(DriverProfile::Modifier_State::SLOW_DRIVE):
    lines [2] = "Slow boi";
    break;

    case(DriverProfile::Modifier_State::NORMAL_DRIVE):
    lines[2] = "Normal boi";
    break;

    default:
    lines[2] = "NULL boi";
    break;
   }



    for(int i = 0; i < lines.size(); i++) {  // For Loop to fill up controller screen string length (15 characters) (to remove default VEXOS UI)

      if(lines.at(i) != lastLines.at(i)) {

        std::string str = lines.at(i);
        if(str.size() < maxWidth) { // Check if string is less than max controller string length (15)
          str.insert(str.end(), maxWidth - str.size(), ' '); // Fill up with spaces if characters unsued
        } else {
          str.erase(str.begin() + maxWidth, str.end()); // Remove characters if over limit
        }

        master_controller.set_text(i, 0, str.c_str()); // Set each chracter on each line indiviually
        master_controller.print(i, 0, str.c_str());

        lastLines.at(i) = lines.at(i); // Check for each cycle if character at (i) has changed
        pros::delay(52); // Delay at max polling rate (50ms)
        delayed = true;
      }

    }

    if(!delayed) {
      pros::delay(52); // if no changes to text, just sleep
    }
  }
}
