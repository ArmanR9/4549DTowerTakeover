#include "main.h"
#include "utilities.hpp"

// pi_arman? really?
const float pi_arman = 3.14159265358;
const float width_to_center = 5.78125; // Width to center of wheel constant (for odometry computation)

// Floating point modulo

float _fmod(float x, float y)
{
	int q = floor(x / y);
	return x - (float)q * y;
}

// Sign function

template <typename T> int sgn_(T val) {
    return (T(0) < val) - (val < T(0));
}

// Good ol' me didn't know how to use templates. Templates are supposed to go in the header.
template int sgn_(int);
template int sgn_(float);
template int sgn_(double);


/* Old sign method
int sgn(double v) {
  return (v < 0) ? -1 : ((v > 0) ? 1 : 0);
}
*/

int _round(float x){
  int z = x - (int)x;

  z = ((z > 0.5) ? 1 : 0);

  return (int)x + z;
}

