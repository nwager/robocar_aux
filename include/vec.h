/**
 * @file vec.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief Simple vector math functions.
 * @version 0.1
 * @date 2022-12-14
 * 
 * @copyright Copyright (c) 2022
 * 
 * Provides some standard vector operations. These are naively implemented
 * and hard-coded to one number of dimensions, so don't expect much.
 * 
 */
#ifndef _VEC_H_
#define _VEC_H_

#include "pico/float.h"

#ifndef NO_PRINT
#include <stdio.h>
#endif // NO_PRINT

/**
 * @brief Sets all elements of a vector to 0.
 * 
 * @param vec Vector to zero out.
 */
void vec_zero(float vec[3]);

/**
 * @brief Returns the sum of two vectors (a+b) in an output vector.
 * 
 * @param a Operand vector 1.
 * @param b Operand vector 2.
 * @param out Output vector.
 */
void vec_add(float a[3], float b[3], float out[3]);

/**
 * @brief Returns the difference of two vectors (a-b) in an output vector.
 * 
 * @param a Operand vector 1.
 * @param b Operand vector 2.
 * @param out Output vector.
 */
void vec_sub(float a[3], float b[3], float out[3]); 

/**
 * @brief Multiply vector by scalar and store in output vector.
 * 
 * @param vec Input vector to scale.
 * @param scalar Scalar value to multiply vector.
 * @param out Output vector.
 */
void vec_scalar_mul(float vec[3], float scalar, float out[3]);

/**
 * @brief Divide vector by scalar and store in output vector.
 * 
 * @param vec Input vector to scale.
 * @param scalar Scalar value to divide vector.
 * @param out Output vector.
 */
void inline vec_scalar_div(float vec[3], float scalar, float out[3]) {
    vec_scalar_mul(vec, 1.0 / scalar, out);
}

/**
 * @brief Returns the arithmetic mean of a list of vectors in an output
 *     vector.
 * 
 * @param vecs List of vectors to average.
 * @param vecs_size Number of vectors in vecs.
 * @param mean Output vector to store mean.
 */
void vec_mean(float vecs[][3], int vecs_size, float mean[3]);

/**
 * @brief Copy vector src elements to vector dst.
 * 
 * @param in Source vector to copy.
 * @param out Destination vector to store copied values.
 */
void vec_copy(float src[3], float dst[3]);

/**
 * @brief Get magnitude of vector (2-norm).
 * 
 * @param vec Vector to measure.
 * @return float Magnitude of vector.
 */
float vec_mag(float vec[3]);


#ifndef NO_PRINT

/**
 * @brief Print vector to stdout (no newline).
 * 
 * @param v Vector to print.
 */
void vec_print(float v[3]);

/**
 * @brief Print vector to stdout (with newline).
 * 
 * @param v Vector to print.
 */
void vec_println(float v[3]);

#endif // NO_PRINT

#endif // _VEC_H_