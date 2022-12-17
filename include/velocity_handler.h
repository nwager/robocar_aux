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

#ifndef _VELOCITY_HANDLER_H_
#define _VELOCITY_HANDLER_H_

#include <limits.h> // for ULONG_MAX

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "type_utils.h"
#include "mpu6050.h"

#define WHEEL_REV_GPIO 15
#define PI 3.14159
#define WHEEL_R_M (49.68 / 1000)
#define WHEEL_C_M (2.0 * PI * WHEEL_R_M)

#define WHEEL_NOTIF_MASK (1 << 0)
#define ACCEL_NOTIF_MASK (1 << 1)

typedef struct {
    QueueHandle_t vel_queue;
    QueueHandle_t accel_queue;
} update_vel_task_arg_t;

/**
 * @brief Updates velocity on wheel revolution interrupt and publishes
 *     to queue.
 * 
 * @param p Pointer to update_vel_task_arg_t.
 */
void update_vel_task(void *p);

#endif // _VELOCITY_HANDLER_H