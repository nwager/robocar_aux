/**
 * @file actuation.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief Actuates vehicle.
 * @version 0.1
 * @date 2022-12-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef ACTUATION_H
#define ACTUATION_H

#include <queue.h>

static const int ESC_PWM_MAX = 1900;
static const int ESC_PWM_ZERO = 1500;
static const int ESC_PWM_MIN = 1100;

static const int SERVO_PWM_MAX = 1770;
static const int SERVO_PWM_ZERO = 1500;
static const int SERVO_PWM_MIN = 1230;

static const float SERVO_MIN_ANGLE = -0.4;
static const float SERVO_MAX_ANGLE = 0.4;

/**
 * @brief How much to left-shift the device ID when formatting actuation
 *     notification value.
 * 
 */
#define ACTU_DEVICE_ID_SHIFT 16

typedef enum {
    DEVICE_ID_ESC = 0,
    DEVICE_ID_SERVO,
    DEVICE_NONE = -1
} actuation_device_id_t;

typedef struct {
    uint value;
    actuation_device_id_t id;
} actuation_item_t;

typedef struct {
    QueueHandle_t control_queue;
    QueueHandle_t vel_queue;
    QueueHandle_t act_queue;
} pid_vel_task_arg_t;

typedef struct {
    QueueHandle_t control_queue;
} steering_task_arg_t;

typedef struct {
    QueueHandle_t act_queue;
} actuation_task_arg_t;

/**
 * @brief Task that controls velocity with PID control.
 * 
 * @param p Pointer to pid_vel_task_arg_t.
 */
void pid_vel_task(void *p);

/**
 * @brief Task that polls control queue to update steering angle.
 * 
 * @param p Pointer to steering_task_arg_t.
 */
void steering_task(void *p);

/**
 * @brief When notified, reads all items from actuation queue and actuates
 *     corresponding devices.
 * 
 * @param p Pointer to actuation_task_arg_t.
 */
void actuation_task(void *p);

/**
 * @brief Converts an angle (radians) to PWM signal to send to the servo.
 * 
 * @param angle Steering angle in radians.
 * @return (uint) PWM microseconds to send to the servo.
 */
uint angle_to_pwm(float angle);

/**
 * @brief Tells the actuation task to actuate the given device with the
 *     given value. Works by adding the command to the queue and notifying
 *     the actuation task.
 * 
 * @param id Device ID to actuate.
 * @param value Actuation value (PWM signal).
 * @param timeout Number of ticks to wait for queue push.
 * @return (true) Successfully added command to the queue, should be processed
 *     momentarily.
 * @return (false) Command failed to send.
 */
bool actuate_device(actuation_device_id_t id,
                    uint value,
                    TickType_t timeout);

#endif /* ACTUATION_H */