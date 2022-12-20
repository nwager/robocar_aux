#include "math_utils.h"

int clampi(int val, int min, int max) {
    return val < min ? min : (val > max ? max : val);
}

float mapf(float in,
           float in_min,
           float in_max,
           float out_min,
           float out_max) {

    return ((in - in_min) / (in_max - in_min))
        * (out_max - out_min) + out_min;
}