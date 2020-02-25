#include "main.h"
#include "utilities.hpp"

const float pi_arman = 3.14159265358;
const float width_to_center = 5.78125;

float _fmod(float x, float y)
{
	int q = floor(x / y);
	return x - (float)q * y;
}

template <typename T> int sgn_(T val) {
    return (T(0) < val) - (val < T(0));
}

template int sgn_(int);
template int sgn_(float);
template int sgn_(double);


int sgn(double v) {
  return (v < 0) ? -1 : ((v > 0) ? 1 : 0);
}

int _round(float x){
int z = x - (int)x;

z = ((z > 0.5) ? 1 : 0);

return (int)x + z;
}

float arman_powf(float x, float y){
for(int g = 0; g < y; g++)
{
x*=g;
}
return x;
}
