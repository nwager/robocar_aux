#ifndef _MPU6050_UTILS_H_
#define _MPU6050_UTILS_H_

#include <stdint.h>

void mpu6050_command(uint8_t reg, uint8_t val);
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
void mpu6050_convert_raw_accel(uint16_t raw[3], float out[3]);

#endif // _MPU6050_UTILS_H_