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

#ifndef _TYPE_UTILS_H_
#define _TYPE_UTILS_H_

#include <stdint.h>

/// @brief Unsigned microseconds.
typedef uint64_t umicros_t;

/**
 * @brief Control message sent to, or received from, master.
 * 
 */
typedef struct { float vel, steer; } control_msg_t;

#endif // _TYPE_UTILS_H_