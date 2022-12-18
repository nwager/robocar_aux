/**
 * @file velocity_handler.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief Receives wheel revolution interrupt and updates velocity.
 * @version 0.1
 * @date 2022-12-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef VELOCITY_HANDLER_H
#define VELOCITY_HANDLER_H

#include <queue.h> // for QueueHandle_t

#define WHEEL_NOTIF_MASK (1 << 0)
#define ACCEL_NOTIF_MASK (1 << 1)

typedef struct {
    QueueHandle_t vel_queue;
    QueueHandle_t accel_queue;
} update_vel_task_arg_t;

/**
 * @brief Updates velocity from wheel revolution and acceleration
 *     measurements and then pushes new velocity to queue.
 *     Blocked unless new measurements are available.
 * 
 * @param p Pointer to update_vel_task_arg_t.
 */
void update_vel_task(void *p);

#endif /* VELOCITY_HANDLER_H */