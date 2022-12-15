/**
 * @file core1_main.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief Defines tasks to be run by Pico core 1.
 * @version 0.1
 * @date 2022-12-13
 * 
 * @copyright Copyright (c) 2022
 * 
 * Core 1 is responsible for interfacing with the accelerometer and
 * wheel rotation sensor (via interrupt). It controls the I2C bus and
 * updates the current perceived speed of the vehicle. This responsibility
 * was delegated to a separate core so the core 0 doesn't have to waste
 * resources managing peripheral communication (except with the computer).
 */

#ifndef _CORE1_MAIN_H_
#define _CORE1_MAIN_H_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/util/queue.h"

#include "mpu6050.h"
#include "type_utils.h"
#include "core_comm_queue.h"

#define I2C_SDA_GPIO 8
#define I2C_SCL_GPIO 9
#define I2C_BAUD_RATE (400 * 1000)

#define PI 3.14159
#define WHEEL_R_M (49.68 / 1000)
#define WHEEL_C_M (2.0 * PI * WHEEL_R_M)

/**
 * @brief Core 0 sends commands to core 1 using this queue with 1-byte
 *     command codes.
 */
extern queue_t command_queue;
/**
 * @brief Core 1 responds to core 0 commands with this queue with void
 *     pointers to data.
 */
extern queue_t response_queue;

/**
 * @brief Entry point for core 1.
 * 
 */
void core1_entry();

/**
 * @brief Set up I2C bus with defined values.
 * 
 */
static void i2c_setup();

/**
 * @brief Initialize MPU6050. Must be called after i2c_setup().
 * 
 */
static void mpu6050_reset();

/**
 * @brief Wheel revolution sensor interrupt handler. Records time of
 *     interrupt so the main thread can calculate speed.
 * 
 * @param _gpio (unused) Which GPIO caused interrupt.
 * @param _event_mask (unused) Which events caused interrupt.
 */
static void wheel_rev_irq_handler(uint _gpio, uint32_t _event_mask);

/**
 * @brief Calculate vehicle speed given a time delta between wheel
 *     revolution readings.
 * 
 * @param dt Time between wheel revolution readings in microseconds.
 * @return float Perceived vehicle speed in m/s.
 */
static inline float calculate_speed(umicros_t dt);

#endif // _CORE1_MAIN_H_
