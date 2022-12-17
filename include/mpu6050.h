/**
 * @file mpu6050.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief Defines useful constants and methods for interfacing with an
 *     MPU6050 via I2C.
 * @version 0.1
 * @date 2022-12-13
 * 
 * @copyright Copyright (c) 2022
 * 
 * Defines MPU6050 register addresses and values to write to them. The
 * methods issue commmands to the MPU6050 and receive data via the pico's
 * I2C bus. I2C0 is used by default, but you can change the I2C bus by
 * defining I2C_INSTANCE as the pico I2C identifier before including this.
 */

#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdio.h>
#include <limits.h> // for ULONG_MAX
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "vec3.h"
#include "type_utils.h"
#include "velocity_handler.h"

#ifndef I2C_INSTANCE
    #define I2C_INSTANCE i2c_default
#endif

#define MPU6050_ADDR 0x68
#define GS_TO_MSS 9.8 // factor to convert from gs to m/s/s

// registers
#define MPU6050_CONFIG 0x1A
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_REG 0x3B
#define MPU6050_PWR_MGMT_1 0x6B

// values
// dlpf frequency cutoffs  (values for accelerometer)
#define MPU6050_DLPF_260HZ 0
#define MPU6050_DLPF_184HZ 1
#define MPU6050_DLPF_94HZ  2
#define MPU6050_DLPF_44HZ  3
#define MPU6050_DLPF_21HZ  4
#define MPU6050_DLPF_10HZ  5
#define MPU6050_DLPF_5HZ   6
// accelerometer ranges
#define MPU6050_ACCEL_RANGE_2G  (0 << 3)
#define MPU6050_ACCEL_RANGE_4G  (1 << 3)
#define MPU6050_ACCEL_RANGE_8G  (2 << 3)
#define MPU6050_ACCEL_RANGE_16G (3 << 3)

/// @brief Stores current accelerometer g-range for calculations.
extern float accel_range;

/// @brief Stores gravity offset vector from calibration.
extern vec3_t gravity_offset;

/// @brief Stores forward direction from calibration (unit vector).
extern vec3_t fwd_dir;

typedef struct {
    control_msg_t *control_state; // current controls
    QueueHandle_t accel_queue; // queue to send acceleration data
} mpu6050_task_arg_t;

/**
 * @brief Task periodically reads MPU6050 accelerometer and updates the
 *     estimated speed.
 * 
 * @param p Pointer to mpu6050_task_arg_t.
 */
void mpu6050_task(void *p);

/**
 * @brief Send a register-value command to MPU6050 over I2C.
 * 
 * @param reg MPU6050 register.
 * @param val Value to write to register.
 * @return true Command was successfully sent.
 * @return false Command failed to send.
 */
bool mpu6050_command(uint8_t reg, uint8_t val);

/**
 * @brief Turn on MPU6050.
 */
void mpu6050_activate();

/**
 * @brief Set the frequency cutoff for the integrated digital low pass
 *     filter (DLPF).
 * 
 * @param freq DLPF frequency cutoff. Must be MPU6050_DLPF_<freq>HZ
 *     where <freq> is one of {5, 10, 21, 44, 94, 184, 260}.
 */
void mpu6050_set_dlpf(uint8_t freq);

/**
 * @brief Set the +/- g-range of the accelerometer.
 * 
 * @param range Range (+/- gs). Must be MPU6050_ACCEL_RANGE_<range>G where
 *     <range> is one of {2, 4, 8, 16}.
 */
void mpu6050_set_range(uint8_t range);

/**
 * @brief Read from the MPU6050 accelerometer and get the measurements as
 *     floats in m/s/s.
 * 
 * @param out Pointer to output vector to store the measurements.
 */
void mpu6050_get_accel(vec3_t *out);

/**
 * @brief Perform calibration procedure to determine gravity offset and
 *     forward direction so readings can be used for linear motion.
 *
 */
void mpu6050_calibrate();

/**
 * @brief Returns the forward component of the given acceleration vector.
 * 
 * @param total_accel Pointer to acceleration vector (no adjustments for
 *     direction or gravity).
 * @return (float) The forward component of the acceleration vector in m/s/s.
 */
float mpu6050_get_fwd_from_total(vec3_t *total_accel);

#endif // _MPU6050_H_