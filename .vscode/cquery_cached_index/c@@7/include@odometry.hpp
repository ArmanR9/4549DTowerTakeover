#ifndef ODOM_HPP
#define ODOM_HPP
#include "main.h"

enum class _TurnDir{
CW,
CCW,
CH
};

struct Vector{
double x;
double y;
};


struct Polar{
double r;
double theta;
};

struct PosUtils{
  //Orientation Terms
  float prevAlpha; // Previous Theta from the previous CPU cycle
  float dAlpha; // Change in Theta per CPU cycle
  float alphaAvg; // Averaged Theta

  float rR; // Turing radius from right wheel to turning center
  float rL; // Turning radius from left wheel to turing center
  float rB; // Turning radius from back wheel to turning center

  float h; // hypotenuse of the displacement we travel
  float hB; // same as h, but for back wheel
  float alpha_2; // alpha / 2 (half our orientation, that creates 2 right angle triangles with h)
  float beta; //the ending orientation of the movement (angle formed by traingle by h) and is a/2)

  float dL; // left wheel travel
  float dR; // right wheel travel
  float dB; //backwheel travel
  float dS; // distance the tracking center travels
};

// TODO:
//Make this into a class with private interface (to avoid accidentally overwriting a,x,y)

class ABSPosition{
private:
  // Data that can't be exposed directly to the public interface
  // Important data that is Read-Only outside tracking.cpp
  double m_alpha;
  double m_a_initial{0.0};
  double m_x;
  double m_y;

public:
  double lW{0.0};
  double rW{0.0};
  double bW{0.0};
  float dA_copy{0.0};

  //Methods

  //Setters
  void set_alpha(double a){ m_alpha = a; }
  void set_a_initial(double b){m_a_initial = b;}
  void set_x(double x){m_x = x;}
  void set_y(double y){m_y = y;}

  // Getters
  double get_alpha(){ return m_alpha;}
  double get_a_initial(){ return m_a_initial;}
  double get_x(){ return m_x;}
  double get_y(){ return m_y;}

  //Tracking math
  void compute_position(PosUtils& o);
  void computeX(PosUtils& o);
  void computeY(PosUtils& o);
  void computeA(PosUtils& o);

  void set_new_pos(double x, double y, double a);
  void reset_pos();

  void log_position();
  // Default constructor
  // Initialize everything to 0
  ABSPosition() {}

};

class Velocity{
  private:
    double m_vel_x;
    double m_vel_y;
    double m_vel_a;

    double m_vel;
    double m_accel;
    double m_jerk;

    double m_last_pos_x;
    double m_last_pos_y;
    double m_last_pos_a;

    double m_last_pos;
    double m_last_vel;
    double m_last_accel;

    uint32_t m_curTime;
    uint32_t m_lastTime;
    uint32_t m_dT;

public:

// Methods
void compute_velocity_xya(ABSPosition& position);
void compute_velo_accel_jerk();

// Setters
void set_vel_x(double vx){ m_vel_x = vx; }
void set_vel_y(double vy){m_vel_y = vy;}
void set_vel_a(double va){m_vel_a = va;}

void set_vel(double v){ m_vel = v; }
void set_accel(double a){m_accel = a;}
void set_jerk(double j){m_jerk = j;}

// Getters
double get_vel_x(){ return m_vel_x;}
double get_vel_y(){ return m_vel_y;}
double get_vel_a(){ return m_vel_a;}

double get_vel(){ return m_vel; }
double get_accel(){ return m_accel;}
double get_jerk(){ return m_jerk;}

void reset_velocity(ABSPosition &position);
void log_velocity();

 Velocity() {}
};


double lineAngle(double endX, double endY, double startX, double startY);

Polar vector_to_polar(Vector& v, Polar& p);

Vector polar_to_vector(Polar& p, Vector& v);

double degrees_to_radians(double degrees);

double radians_to_degrees(double radians);

double nearAngle(double angle, double reference);

extern ABSPosition pos;
extern Velocity velo;

void reset_pos();
//void odom_task(void* ignore);
void tracking_update(void* ignore);

#endif
