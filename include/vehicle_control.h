/**
 * @file vehicle_control.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief PID control task for controlling velocity.
 * @version 0.1
 * @date 2022-12-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <queue.h>

#define PWM_MAX 1900
#define PWM_ZERO 1500
#define PWM_MIN 1100

typedef struct {
    QueueHandle_t control_queue;
    QueueHandle_t vel_queue;
} pid_vel_task_arg_t;

/**
 * @brief Task that controls velocity with PID control.
 * 
 * @param p Pointer to pid_vel_task_arg_t.
 */
void pid_vel_task(void *p);

/**
 * @brief How much to left-shift the device ID when formatting actuation
 *     notification value.
 * 
 */
#define ACTU_DEVICE_ID_SHIFT 16
typedef enum {
    DEVICE_ESC = 0,
    DEVICE_SERVO
} actu_device_id_t;

/**
 * @brief Receives notification messages with device ID and PWM value
 *     and actuates that device with the notification PWM signal.
 * 
 *     The notification value is formatted as such:
 *         16 MSB device ID, 16 LSB PWM value
 * 
 * @param p NULL
 */
void actuation_task(void *p);

uint32_t format_actuation_notif(actu_device_id_t id, uint pwm);

#endif /* VEHICLE_CONTROL_H */