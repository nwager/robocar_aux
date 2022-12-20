/**
 * @file math_utils.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief General math constants and functions.
 * @version 0.1
 * @date 2022-12-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#define PI 3.14159265359

/**
 * @brief Clamp integer value to the range [min, max]
 * 
 * @param val Value to clamp.
 * @param min Minimum value in range.
 * @param max Maximum value in range.
 * @return (int) val if min <= val <= max, min if val < min, max if val > max.
 */
int clampi(int val, int min, int max);

/**
 * @brief Given an input range [in_min, in_max], linearly map input value
 *     to [out_min, out_max].
 * 
 * @param in Value to map.
 * @param in_min Minimum value of input range.
 * @param in_max Maximum value of input range.
 * @param out_min Minimum value of output range.
 * @param out_max Maximum value of output range.
 * @return (float) in value mapped to output range.
 */
float mapf(float in,
           float in_min,
           float in_max,
           float out_min,
           float out_max);

#endif /* MATH_UTILS_H */