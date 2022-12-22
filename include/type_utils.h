/**
 * @file type_utils.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief General type declarations.
 * @version 0.1
 * @date 2022-12-14
 * 
 * @copyright Copyright (c) 2022
 * 
 * Useful types that don't fit in any one module.
 * 
 */

#ifndef TYPE_UTILS_H
#define TYPE_UTILS_H

#include <stdint.h>

#include "pico/stdlib.h"

#include "vec3.h"

/**
 * @brief Unsigned microseconds.
 * 
 */
typedef uint64_t umicros_t;

/**
 * @brief Control message sent to, or received from, brain.
 * 
 */
typedef struct { float vel, steer; } control_msg_t;

/**
 * @brief Acceleration vector and the system time it was measured.
 * 
 */
typedef struct {
    absolute_time_t time;
    vec3_t accel;
} accel_stamped_t;


/**
 * @brief Velocity and the system time it was measured.
 * 
 */
typedef struct {
    absolute_time_t time;
    float vel;
} velocity_stamped_t;

#endif /* TYPE_UTILS_H */