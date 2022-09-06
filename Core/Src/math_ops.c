
#include "math_ops.h"
#include "lookup.h"


float fast_fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

float fast_fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

float fmaxf3(float x, float y, float z){
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }

float fminf3(float x, float y, float z){
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }
/*
float roundf(float x){
    /// Returns nearest integer ///
    return x < 0.0f ? ceilf(x - 0.5f) : floorf(x + 0.5f);
    }
  */
void limit_norm(float *x, float *y, float limit){
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrtf(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x * limit/norm;
        *y = *y * limit/norm;
        }
    }
    
void limit(float *x, float min, float max){
    *x = fast_fmaxf(fast_fminf(*x, max), min);
    }




float sin_lut(float theta){
	theta = fmodf(theta, TWO_PI_F);
	theta = theta<0 ? theta + TWO_PI_F : theta;

	return sin_tab[(int) (LUT_MULT*theta)];
}

float cos_lut(float theta){
	return sin_lut(PI_OVER_2_F - theta);
}

int float_to_uint_internal(float x, const float x_max, unsigned int bits, int symmetric)
{
    float span = x_max;
    if(symmetric){
    	span += x_max;
    }
    float quantum = span/((1<<bits)-1);

    float offset = 0;
    if (symmetric) {
        offset = -x_max - quantum/2;
    }

    return (int) ((x-offset)/quantum + 0.5);
}

float uint_to_float_internal(int x, const float x_max, unsigned int bits, int symmetric)
{
    float span = x_max;
    if(symmetric){
    	span += x_max;
    }
    float quantum = span/((1<<bits)-1);

    float offset = 0;
    if (symmetric) {
        offset = -x_max - quantum/2;
    }

    return ((float)x)*quantum + offset;
}

int float_to_uint_symmetric(float x, const float x_max, unsigned int bits){
	return float_to_uint_internal(x, x_max, bits, 1);
}

int float_to_uint_positive(float x, const float x_max, unsigned int bits){
	return float_to_uint_internal(x, x_max, bits, 0);
}

float uint_to_float_symmetric(int x, const float x_max, unsigned int bits){
	return uint_to_float_internal(x, x_max, bits, 1);
}

float uint_to_float_positive(int x, const float x_max, unsigned int bits){
	return uint_to_float_internal(x, x_max, bits, 0);
}


//Ben's original functions, replaced above.
/*
int float_to_uint_old(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float_old(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

*/



