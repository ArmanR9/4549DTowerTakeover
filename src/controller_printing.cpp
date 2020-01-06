#include "controller_printing.hpp"
#include "main.h"
#include <string>
#include <array>
#include "sensors.hpp"
#include "motors.hpp"
#include "joystick.hpp"
using namespace pros;

std::array<std::string, 3> lines = {};
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

    switch(profile->getDrive()){
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

     switch(profile->getCurve()){
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

   switch(profile->get_sModifier()){

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


    //while(!pros::competition::is_autonomous()){

    for(int i = 0; i < lines.size(); i++) {

      if(lines.at(i) != lastLines.at(i)) {

        std::string str = lines.at(i);
        if(str.size() < maxWidth) {
          str.insert(str.end(), maxWidth - str.size(), ' ');
        } else {
          str.erase(str.begin() + maxWidth, str.end());
        }


/*          if(lines.at(i) == lines.at(1)){
            master_controller.print(i, 0 , str.c_str(), opc_profile.get_profile());
          }

          else if(lines.at(i) == lines.at(3)){
            master_controller.print(i, 0, str.c_str(), pros::battery::get_capacity());
          }

        */    master_controller.set_text(i, 0, str.c_str());
        master_controller.print(i, 0, str.c_str());
      //  pros::c::controller_set_text(pros::E_CONTROLLER_MASTER, i, 0, str.c_str());

        lastLines.at(i) = lines.at(i);
        pros::delay(52);
        delayed = true;
      }

    }
//  }
    if(!delayed) {
      pros::delay(52);
    }
  }
}
