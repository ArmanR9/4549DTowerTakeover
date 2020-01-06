#include "main.h"
#include "odometry.hpp"
#include "utilities.hpp"
#include "motors.hpp"
#include "sensors.hpp"

/*void odom_task(void* ignore){
  //std::uint32_t timer = pros::millis();
  std::uint32_t timer = pros::millis();

  while(true){
    tracking_update();

    pros::Task::delay_until(&timer, 10);
    }

}
*/

//Polar p;
ABSPosition pos;
Velocity velo;

void tracking_update(void *){
std::uint32_t timer = pros::millis();
PosUtils odom;

pos.set_alpha(pos.get_a_initial());

while(true){

  odom.dL = (LB_mtr.get_position() - pos.lW) * ticks_to_in_mtr;
  //dL = (encoder360L.get_value() - pos.lW) * TICKS_TO_IN;
  odom.dR = (RB_mtr.get_position() - pos.rW) * ticks_to_in_mtr;
  //dR = (encoder360R.get_value() - pos.rW)* TICKS_TO_IN;
  odom.dB = (encoder360B.get_value() - pos.bW) * ticks_to_in;

  odom.dS = odom.dL + odom.dR / 2.0;

  pos.lW = LB_mtr.get_position();
  //pos.lW = encoder360L.get_value();
  pos.rW = RB_mtr.get_position();
  //pos.rW = encoder360R.get_value();
  pos.bW = encoder360B.get_value();


 // This uses the arc length formula to find the heading given in radians for the bot.
 // Using the terms we know (the arc length which is eO and eI, and then the radius to the arc angle, we can find the angle that the robot turns)
 // computes in radians

odom.dAlpha = ((odom.dL - odom.dR) / (chassis_width));


  if(odom.dAlpha){ // If Delta Theta is 0, there is no rotational movement and is strictly translational

    odom.alpha_2 = odom.dAlpha / 2.0;

    odom.rR = odom.dR / odom.dAlpha + r_distance_in;
    odom.rL = odom.dL / odom.dAlpha - l_distance_in;
    odom.rB = odom.dB / odom.dAlpha + b_distance_in;

    odom.h = (odom.rR * sin(odom.alpha_2)) * 2.0; // Calculate the hypotenuse of our movement.
    // The displacement of our movement using 2 symmetric right angle triangles (alpha/2) then multiplying by 2.

    odom.hB = (odom.rB * sin(odom.alpha_2)) * 2.0; // Same as h, but for the back wheel

    }
else{
  odom.alpha_2 = 0.0; // alpha / 2 is 0, since theta is 0.
  odom.h = odom.dS; // displacement is simply how much the center the bot moved, as there is no arc.
  odom.hB = odom.dB; // back wheel displacement when there is no arc.
}


  odom.beta = odom.alpha_2 + pos.get_alpha(); // The ending angle beta of a right angle triangle (calculated from the complimentary angle from a right-angle (a/2) )

  odom.alphaAvg = odom.prevAlpha + odom.alpha_2; // Average Alpha placeholder



 odom.prevAlpha = odom.dAlpha; // Alpha from last CPU cycle

 pos.dA_copy = odom.dAlpha; // used for logging function

/* pos.computeX(odom);
 pos.computeY(odom);
 pos.computeA(odom);
 */

 pos.compute_position(odom);
 // Updating my global position from computing x,y,a from last cycle


  //Velocity tracking
  velo.compute_velocity(pos);


  pros::Task::delay_until(&timer, 10);
 }
}

void ABSPosition::compute_position(PosUtils& o){
m_x += o.h * sin(o.beta);
m_x += o.hB * cos(o.beta);

m_y += o.h * cos(o.beta);
m_y += o.hB * sin(-o.beta);

m_alpha += o.dAlpha;
}


void ABSPosition::computeX(PosUtils& o){
m_x += o.h * sin(o.beta);
m_x += o.hB * cos(o.beta);
}

void ABSPosition::computeY(PosUtils& o){
m_y += o.h * cos(o.beta);
m_y += o.hB * sin(-o.beta);
}

void ABSPosition::computeA(PosUtils& o){
m_alpha += o.dAlpha;
}



void ABSPosition::set_new_pos(double x, double y, double a){
m_x = x;
m_y = y;
m_alpha = a;
}


void ABSPosition::reset_pos(){
  	lW = rW = bW = 0.0;
  	m_x = m_y = m_alpha = 0.0;
}

void ABSPosition::log_position(){
  std::cout << "a: " << m_alpha <<'\n';
  std::cout << "dA: " << dA_copy <<'\n';
  std::cout << "X: " << m_x <<'\n';
  std::cout << "Y :" << m_y << '\n';
}

// ********************  //
// **** Velocity ****** //
// *********************//
// **** Velocity ***** //
// ******************* //

void Velocity::compute_velocity(ABSPosition& position){
  m_curTime = pros::millis();
  m_dT = m_curTime - m_lastTime;

    if(m_dT > 40){
    m_vel_x = (position.get_x() - m_last_pos_x) / (static_cast<float>(m_dT) / 1000.0);
    m_vel_y = (position.get_y() - m_last_pos_y) / (static_cast<float>(m_dT) / 1000.0);
    m_vel_a = (position.get_alpha() - m_last_pos_a) / (static_cast<float>(m_dT) / 1000.0);

     m_last_pos_x = position.get_x();
     m_last_pos_y = position.get_y();
     m_last_pos_a = position.get_alpha();

     m_lastTime = m_curTime;
    }

 }

void Velocity::reset_velocity(ABSPosition& position){
m_vel_x = m_vel_y = m_vel_a = 0.0;

m_last_pos_x = position.get_x();
m_last_pos_y = position.get_y();
m_last_pos_a = position.get_alpha();
 m_lastTime = m_curTime;
}

void Velocity::log_velocity(){
  std::cout << "vX: " << m_vel_x <<'\n';
  std::cout << "vY: " << m_vel_y <<'\n';
  std::cout << "vA: " << m_vel_a <<'\n';
}


// ********************  //
// **** Utilities ***** //
// *********************//
// **** Utilities **** //
// ******************* //

double lineAngle(double endX, double endY, double startX, double startY){
return atan2(endX - startX, endY - startY);
}

Polar vector_to_polar(Vector& v, Polar& p){

if(v.x || v.y){
p.r = sqrt(v.x * v.x + v.y * v.y);
p.theta = atan2(v.x, v.y);
}

else p.r = p.theta = 0;

return p;
}

Vector polar_to_vector(Polar& p, Vector& v){
if(p.r){
v.x = cos(p.theta) * p.r;
v.y = sin(p.theta) * p.r;
}
else v.x = v.y = 0;

return v;
}



double degrees_to_radians(double degrees)
{
	return degrees * M_PI / 180.0;
}

double radians_to_degrees(double radians)
{
	return radians * 180.0 / M_PI;
}

double nearAngle(double angle, double reference)
{
	return round((reference - angle) / (2.0 * M_PI)) * (2.0 * M_PI) + angle;
}
