#include "main.h"
#include "joystick.hpp"
#include "motors.hpp"
#include "sensors.hpp"
#include "utilities.hpp"

void DriverProfile::toggle_switch(char btn){

  // Toggle feature
  if(!get_isPressed())
    {
      set_toggle(1 - get_toggle());

      set_isPressed(1);
      pros::delay(200);
    }
  else
  {
    set_isPressed(0);
    pros::delay(200);
  }


  // Arcade/Tank switching
    if(btn == 'L'){

      if(get_toggle())
         setDrive(Drive_State::ARCADE_DRIVE);

         else if(!get_toggle())
            setDrive(Drive_State::TANK_DRIVE);
    }


  // Modifier switching
    else if(btn == 'R'){

       if(get_toggle()){
         setModifier(0.5);
         set_sModifier(Modifier_State::SLOW_DRIVE);
       }

       else if(!get_toggle()){
         setModifier(1);
         set_sModifier(Modifier_State::NORMAL_DRIVE);
       }
     }



    // Curve switching
    else if(btn == 'D'){
      if(get_toggle())
       setCurve(Curve_State::QUADRATIC_DRIVE);

    else if(!get_toggle())
       setCurve(Curve_State::CUBIC_DRIVE);
          }
  }


/*----------------------- /
/ Analog Control Methods  /
/-----------------------*/

int Analog_Control::deadband(int joystick_value){
     if(abs(joystick_value) < 3){
       return 0;
       }
     else{
       return 1;
       }
   }


int Analog_Control::speedCap(int control_value){

  m_max_speed = abs(control_value);

    if(m_max_speed > 120){
      control_value = m_max_speed * sgn_(control_value);
      return control_value;//127 * control_value / max_speed;
      }
      else{
  return control_value;
  }
}


int Analog_Control::calc_l_difference(int & deltaL){
  //m_difference = deltaL;
  m_joy_difference = (master_controller.get_analog(ANALOG_LEFT_Y) - master_controller.get_analog(ANALOG_RIGHT_Y));

    if(m_joy_difference < 15 && m_joy_difference > 0 && abs(getJoyLY(master_controller)) > 20){
      m_assist_control = deltaL;
      }
      else {
        m_assist_control = 0;
      }
return m_assist_control;
}


int Analog_Control::calc_r_difference(int &deltaR){
//  m_difference = deltaR;
  m_joy_difference = (master_controller.get_analog(ANALOG_RIGHT_Y) - master_controller.get_analog(ANALOG_LEFT_Y));

  if(m_joy_difference < 15 && m_joy_difference > 0 && abs(getJoyRY(master_controller)) > 20){
      m_assist_control = deltaR;
      } else {
        m_assist_control = 0;
      }

return m_assist_control;
}

/*-----------------------------
-                             -
-   Joystick drive function   -
-                             -
-----------------------------*/

void joystick_drive(DriverProfile * profile, Analog_Control * analog){

  int delta_joyL = getJoyLY(master_controller) - getJoyRY(master_controller);
  int delta_joyR = getJoyRY(master_controller) - getJoyLY(master_controller);

    switch(profile->getDrive()){

      case (DriverProfile::Drive_State::ARCADE_DRIVE):

        analog->powerLY = pow(master_controller.get_analog(ANALOG_LEFT_Y), 3);
        analog->powerLX = analog->speedCap(master_controller.get_analog(ANALOG_LEFT_X)) * analog->deadband(master_controller.get_analog(ANALOG_LEFT_X));


        leftdrive_set(analog->powerLY + analog->powerLX);
        rightdrive_set(analog->powerLY - analog->powerLX);

        break;



      case (DriverProfile::Drive_State::ARCADE_LIFT_DRIVE):

  // test this multiplying the control exponential
      analog->powerLY = 1.1*(pow(master_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 3)/16129);


   // DEFAULT TANK DRIVE, the normal one

     case (DriverProfile::Drive_State::TANK_DRIVE):


         analog->powerLY = (pow (getJoyLY(master_controller), 2) / 127) * sgn_(getJoyLY(master_controller));
         analog->powerLY -= analog->calc_l_difference(delta_joyL) * sgn_(getJoyLY(master_controller));
         analog->powerLY *= analog->deadband(getJoyLY(master_controller));

           leftdrive_set((analog->speedCap(analog->powerLY))*profile->getModifier());


         analog->powerRY = (pow (getJoyRY(master_controller), 2) / 127) * sgn_(getJoyRY(master_controller));
         analog->powerRY -= analog->calc_r_difference(delta_joyR) * sgn_(getJoyRY(master_controller));
         analog->powerRY *= analog->deadband(getJoyRY(master_controller));

           rightdrive_set((analog->speedCap(analog->powerRY))*profile->getModifier());

      break;
    }
}





/* ---------------- *
*                   *
* Logging Functions *
*                   *
------------------- */



void ToggleClass::logToggleStatus(){
  printf("toggle %d\n", m_toggle);
  printf("isPressed %d\n", m_isPressed);
}


void DriverProfile::logDriverProfile(){
  printf("profile %d\n", get_iDrive());
  printf("drive curve %d\n", get_iCurve());
}


void Analog_Control::logAnalog(){
   printf("leftcontrol %d\n", powerLY - calc_l_difference(powerLY));
   printf("rightcontrol %d\n\n", powerRY - calc_r_difference(powerRY));
   printf("l_difference %d\n", getJoyLY(master_controller) - getJoyRY(master_controller));
   printf("r_difference %d\n", getJoyRY(master_controller) - getJoyLY(master_controller));

   std::cout << "assistL" << calc_l_difference(powerLY) << std::endl;
   std::cout << "assistR" << calc_r_difference(powerRY) << std::endl;
   printf("right joystick %d\n", getJoyLY(master_controller));
   printf("left joystick %d\n",  getJoyRY(master_controller));
}
