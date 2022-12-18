/**
 * @file vec3.h
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
#ifndef VEC_H
#define VEC_H

typedef struct { float v[3]; } vec3_t;

/**
 * @brief Sets all elements of a vector to 0.
 * 
 * @param vec Pointer to vector to zero out.
 */
void vec_zero(vec3_t *vec);

/**
 * @brief Returns the sum of two vectors (a+b) in an output vector.
 * 
 * @param a Pointer to operand vector 1.
 * @param b Pointer to operand vector 2.
 * @param out Pointer to output vector.
 */
void vec_add(vec3_t *a, vec3_t *b, vec3_t *out);

/**
 * @brief Returns the difference of two vectors (a-b) in an output vector.
 * 
 * @param a Pointer to operand vector 1.
 * @param b Pointer to operand vector 2.
 * @param out Pointer to output vector.
 */
void vec_sub(vec3_t *a, vec3_t *b, vec3_t *out); 

/**
 * @brief Multiply vector by scalar and store in output vector.
 * 
 * @param vec Pointer to input vector to scale.
 * @param scalar Scalar value to multiply vector.
 * @param out Pointer to output vector.
 */
void vec_scalar_mul(vec3_t *vec, float scalar, vec3_t *out);

/**
 * @brief Divide vector by scalar and store in output vector.
 * 
 * @param vec Pointer to input vector to scale.
 * @param scalar Scalar value to divide vector.
 * @param out Pointer to output vector.
 */
void inline vec_scalar_div(vec3_t *vec, float scalar, vec3_t *out) {
    vec_scalar_mul(vec, 1.0 / scalar, out);
}

/**
 * @brief Returns the arithmetic mean of a list of vectors in an output
 *     vector.
 * 
 * @param vecs Array of vectors to average.
 * @param vecs_size Number of vectors in vecs.
 * @param mean Pointer to output vector to store mean.
 */
void vec_mean(vec3_t vecs[], int vecs_size, vec3_t *mean);

/**
 * @brief Copy vector src elements to vector dst.
 * 
 * @param in Pointer to source vector to copy.
 * @param out Pointer to destination vector to store copied values.
 */
void vec_copy(vec3_t *src, vec3_t *dst);

/**
 * @brief Get magnitude of vector (2-norm).
 * 
 * @param vec Pointer to vector to measure.
 * @return (float) Magnitude of vector.
 */
float vec_mag(vec3_t *vec);

/**
 * @brief Returns dot product of two vectors <a, b>.
 * 
 * @param a Pointer to operand vector 1.
 * @param b Pointer to operand vector 2.
 * @return (float) Dot product of a and b.
 */
float vec_dot(vec3_t *a, vec3_t *b);

/**
 * @brief Returns resultant vector from projecting a onto b.
 * 
 * @param a Pointer to vector to be projected.
 * @param b Pointer to vector on which a is projected.
 * @param out Output vector.
 */
void vec_project(vec3_t *a, vec3_t *b, vec3_t *out);

/**
 * @brief Returns resultant vector from projecting v onto the given unit
 *     vector. This lets us skip one dot product operation.
 * 
 * @param v Pointer to vector to be projected.
 * @param unit Pointer to unit vector on which to be projected.
 * @param out Output vector.
 */
void vec_project_unit(vec3_t *v, vec3_t *unit, vec3_t *out);

#ifndef NO_PRINT

/**
 * @brief Print vector to stdout (no newline).
 * 
 * @param v Pointer to vector to print.
 */
void vec_print(vec3_t *v);

/**
 * @brief Print vector to stdout (with newline).
 * 
 * @param v Pointer to vector to print.
 */
void vec_println(vec3_t *v);

#endif // ifndef NO_PRINT

#endif /* VEC_H */