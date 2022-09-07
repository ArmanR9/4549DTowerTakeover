#include "main.h"
#include "utilities.hpp"

const float width_to_center = 5.78125; // Width to center of wheel constant (for odometry computation)

// Floating point modulo

float fmod(float x, float y)
{
	int q = floor(x / y);
	return x - (float)q * y;
}

// Sign function

template <typename T> int sgn_(T val) {
    return (T(0) < val) - (val < T(0));
}

// Should probably put this in the header so I don't have to make these instantiations
template int sgn_(int);
template int sgn_(float);
template int sgn_(double);


/* Old sign method
int sgn(double v) {
  return (v < 0) ? -1 : ((v > 0) ? 1 : 0);
}
*/

int round(float x){
  int z = x - (int)x;

  z = ((z > 0.5) ? 1 : 0);

  return (int)x + z;
}

