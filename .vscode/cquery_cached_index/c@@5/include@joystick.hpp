#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include "main.h"


// Parent class solely for toggle functionality
class ToggleClass{
  private:
    // Private member variables

    bool m_isPressed;
    bool m_toggle;


  public:
    // Methods

    void set_isPressed(bool a){ m_isPressed = a; }
    void set_toggle(bool t){ m_toggle = t; }

    bool get_isPressed(){ return m_isPressed; }
    bool get_toggle(){ return m_toggle; }


    void logToggleStatus();

    // Default ctor
    ToggleClass(): m_isPressed(false), m_toggle(false){}
};


class DriverProfile : public ToggleClass {
 public:

   enum class Drive_State : int{
     TANK_DRIVE = 1,
     ARCADE_DRIVE = 2,
     ARCADE_LIFT_DRIVE = 3
   };

   enum class Curve_State : int{
     CUBIC_DRIVE = 1,
     QUADRATIC_DRIVE = 2,
   };

   enum class Modifier_State : int{
    NORMAL_DRIVE = 1,
    SLOW_DRIVE = 2,
  };



  private:
    //Private member variables

    float m_modifier;
    Modifier_State m_sModifier;
    Drive_State m_drive;
    Curve_State m_curve;

  public:

    // Methods

    void setModifier(float m){ m_modifier = m; }
    void setDrive(Drive_State d){ m_drive = d; }
    void setCurve(Curve_State c){ m_curve = c; }
    void set_sModifier(Modifier_State s){m_sModifier = s;}

    float getModifier(){ return m_modifier; }
    Modifier_State get_sModifier(){return m_sModifier;}
    Drive_State getDrive(){ return m_drive; }
    Curve_State getCurve(){ return m_curve; }

    int get_iDrive(){ return static_cast<int>(m_drive);}
    int get_iCurve(){ return static_cast<int>(m_curve); }


//static_cast<int>(m_curve)

    void logDriverProfile();

    // Method that deals with switching driver layout and analog curve
    void toggle_switch(char btn);

    // Constructors

    DriverProfile(Drive_State a, Curve_State b, Modifier_State c) : m_drive(a), m_modifier(1), m_curve(b), m_sModifier(c){} // preferred constructor
    DriverProfile() : m_drive(Drive_State::TANK_DRIVE), m_modifier(1), m_curve(Curve_State::CUBIC_DRIVE), m_sModifier(Modifier_State::NORMAL_DRIVE){} // default constructor

  };


  class Analog_Control{
  private:
    // Private member variable

    int m_assist_control;

    int m_joy_difference;

    int m_max_speed;
  public:

    int powerLX;
    int powerLY;
    int powerRX;
    int powerRY;

  public:
     //Methods



     int deadband(int joystick_value);

     int speedCap(int control_value);

     void logAnalog();

     int calc_l_difference(int & deltaL);
     int calc_r_difference(int & deltaR);


     // Default ctor

     Analog_Control() :
     powerLX(0),
     powerLY(0),
     powerRX(0),
     powerRY(0),

     m_assist_control(0),
  //   m_difference(0),
     m_joy_difference(0),
     m_max_speed(0)
     {}

  };



void joystick_drive(DriverProfile * profile, Analog_Control * analog);
void toggle_switch(DriverProfile * profile, char btn);


#endif
