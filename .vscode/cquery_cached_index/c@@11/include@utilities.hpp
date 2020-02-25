#ifndef UTILS_HPP
#define UTILS_HPP
#include "main.h"

// *----------------------------------- *
// *                                    *
// * Constant macros for algorithims    *
// *                                    *
// *------------------------------------*

// The diameter of the tracking wheels in inches

inline constexpr double wheel_diam_in {2.9883}; // 2.75" tracking wheels // 2.83
inline constexpr float wheel_diam_in_mtr {4.125}; // 4.125" wheels

// The wheel track of the robot (distance between 2 tracking wheels)
inline constexpr double chassis_width {4.23252721};

// The distance between the tracking wheels and the centre of the robot in inches
inline constexpr double l_distance_in {2.22875};
inline constexpr double r_distance_in {2.22875};
inline constexpr float b_distance_in {1.0};

// Ticks for one full revolution of the wheel
inline constexpr double rev_per_tick {360.0};
inline constexpr float rev_per_tick_mtr {900.0};

// Circumference of 2.75 (tracking) and 4.125" (regular) wheels
inline constexpr double circ {2.843 * M_PI}; // 2.783" tracking wheels
inline constexpr double circ_mtr {4.125 * M_PI}; // 4.125" regular wheels

// Comverting raw encoder ticks into inches travelled
inline constexpr double ticks_to_in { (2.611125 * M_PI) / 360.0};
inline constexpr double ticks_to_in_mtr {circ_mtr / rev_per_tick_mtr};

inline constexpr int light_sensor_threshold {1800}; // When lower than 1800 cube is
inline constexpr int light_sensor_threshold2 {2500};
inline constexpr int light_sensor_threshold3 {2400};


// *----------------------------------- *
// *                                    *
// *   Mischanelous math/functions      *
// *                                    *
// *------------------------------------*

float _fmod(float x, float y);

template <typename T> int sgn_(T val);

int sgn(double v);

int _round(float x);

#endif
