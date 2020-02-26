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
Velocity wheel_vel;

void tracking_update(void * ign){
std::uint32_t timer = pros::millis();
PosUtils odom;
pos.reset_pos();

while(true){

  odom.dL = (encoder360L.get_value() - pos.lW) * (ticks_to_in);
  //dL = (encoder360L.get_value() - pos.lW) * TICKS_TO_IN;
  odom.dR = (encoder360R.get_value()  - pos.rW) * (ticks_to_in);
  //dR = (encoder360R.get_value() - pos.rW)* TICKS_TO_IN;
  odom.dB = (encoder360B.get_value() - pos.bW) * (ticks_to_in);

  odom.dS = odom.dL + odom.dR / 2.0;

  pos.lW = encoder360L.get_value();
  //pos.lW = encoder360L.get_value();
  pos.rW = encoder360R.get_value();
  //pos.rW = encoder360R.get_value();
  pos.bW = encoder360B.get_value();


 // This uses the arc length formula to find the heading given in radians for the bot.
 // Using the terms we know (the arc length which is eO and eI, and then the radius to the arc angle, we can find the angle that the robot turns)
 // computes in radians

//Angular Velocity
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

    else {
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


 // Updating my global position from computing x,y,a from last cycle
  pos.compute_position(odom);

  //Velocity tracking
  velo.compute_velocity_xya(pos);

  // For sweep turns
  wheel_vel.compute_velocity_wheels();

///  pos.log_position();
 // std::cout << "L" << encoder360L.get_value() << std::endl;
// std::cout << "R" << encoder360R.get_value() << std::endl;

<<<<<<< HEAD
  pros::delay(10);

 //pros::Task::delay_until(&timer, 10);
=======
  //pros::delay(10);

 pros::Task::delay_until(&timer, 10);
>>>>>>> 92fd986464b0fdbbc43183bc402b23742a241a64
  }
}

void ABSPosition::compute_position(PosUtils& o){
m_x += o.h * sin(o.beta); // Multiply h by sin of our ending engle to get x
m_x += o.hB * cos(o.beta); // Same as above but for back arc

m_y += o.h * cos(o.beta); // Multiply h by cos of our ending engle to get y
m_y += o.hB * -sin(o.beta); // Same as above but for back arc (sin- for rotation matrix)

m_alpha += o.dAlpha; // Add the displacement of alpha to our current alpha

m_angularV = o.dAlpha; // Angular displacement of the robot

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

void Velocity::compute_velocity_xya(ABSPosition& position){
  m_curTime = pros::millis(); // Current time
  m_dT = m_curTime - m_lastTime; // Calculate the time different between cycles

    if(m_dT > 40){
    float pos_a = position.get_alpha();
    float pos_x = position.get_x();
    float pos_y = position.get_y();

    m_vel_x = (pos_x - m_last_pos_x) / (m_dT / 1000.0); // Velocity is ΔdX over Δt (conversion from ms to s)
    m_vel_y = (pos_y - m_last_pos_y) / (m_dT / 1000.0); // Velocity is ΔdY over Δt (conversion from ms to s)
    m_vel_a = (pos_a - m_last_pos_a) / (m_dT / 1000.0); // Velocity is ΔdA over Δt (conversion from ms to s)

     m_last_pos_x = pos_x;
     m_last_pos_y = pos_y; // Get the last positions from last cycle.
     m_last_pos_a = pos_a;

     m_lastTime = m_curTime; // Get the last time from last cycle
    }

 }

 void Velocity::compute_velocity_wheels(){
   m_curTime = pros::millis(); // Current time
   m_dT = m_curTime - m_lastTime; // Calculate the time different between cycles

     if(m_dT > 40){

      float lW = LB_mtr.get_position() * (4.125 * M_PI / 900.0);
      float rW = RB_mtr.get_position() * (4.125 * M_PI / 900.0);

     m_vel_L = (lW - m_last_pos_L) / (m_dT / 1000.0); // Velocity is ΔdX over Δt (conversion from ms to s)
     m_vel_R = (rW - m_last_pos_R) / (m_dT / 1000.0); // Velocity is ΔdY over Δt (conversion from ms to s)

      m_last_pos_R = rW;
      m_last_pos_L = lW; // Get the last positions from last cycle.

      m_lastTime = m_curTime; // Get the last time from last cycle
     }
 }


void Velocity::compute_velo_accel_jerk(){
  m_curTime = pros::millis(); // Current time
  m_dT = m_curTime - m_lastTime; // Calculate the time different between cycles

    if(m_dT > 40){ // Calculate every 40ms
    m_vel = (encoder360avg_in() - m_last_pos) / (m_dT / 1000.0); // Velocity is Δd over Δt (conversion from ms to s)
    m_accel = (m_vel - m_last_vel) / (m_dT / 1000.0); // Acceleration is Δv over Δt (conversion from ms to s)
    m_jerk = (m_accel - m_last_accel) / (m_dT / 1000.0); // Jerk is Δa over Δt (conversion from ms to s)

     m_last_pos = encoder360avg_in();
     m_last_vel = m_vel;    // Get the last velocity, acceleration, and jerk from last cycle.
     m_last_accel = m_accel;

     m_lastTime = m_curTime; // Get the last time from last cycle
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

double getLineAngle(Path iLine){
return atan2(iLine.p2.x - iLine.p1.x, iLine.p2.y - iLine.p1.y);
}

double getLineLength(Path iLine){
float x = iLine.p2.x - iLine.p1.x;
float y = iLine.p2.y - iLine.p1.y;

return sqrt(x * x + y * y);
}


Polar vector_to_polar(Vector& v, Polar& p){

if(v.x || v.y){
p.r = sqrt(v.x * v.x + v.y * v.y);
p.theta = atan2(v.y, v.x);
}

else { p.r = p.theta = 0; }

return p;
}


Vector polar_to_vector(Polar& p, Vector& v){
if(p.r){
v.x = cos(p.theta) * p.r;
v.y = sin(p.theta) * p.r;
}
else { v.x = v.y = 0; }

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

void applyHarshStop(){
    Vector vel;
    vel.x = velo.get_vel_x();
    vel.y = velo.get_vel_y();

  	Polar polarVel;

  	vector_to_polar(vel, polarVel);
  	polarVel.theta += pos.get_alpha();
  	polar_to_vector(polarVel, vel);


  	float yPow = vel.y, aPow = velo.get_vel_a();

    int left = yPow + aPow, right = yPow - aPow;

    left = sgn_(left) * std::max(7, std::abs(left));
    right = sgn_(right) * std::max(7, std::abs(right));

    left = std::min(30, left);
    right = std::min(30, right);

    driveLR_set(left, right);
    pros::delay(150);
    driveLR_set(0, 0);

}
