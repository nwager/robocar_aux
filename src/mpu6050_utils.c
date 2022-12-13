#include "mpu6050_utils.h"
#include "hardware/i2c.h"

#ifndef I2C_INSTANCE
    #define I2C_INSTANCE i2c_default
#endif

#define MPU6050_ADDR 0x68
#define ACCEL_POS_RANGE 8
#define GS_TO_MSS 9.8

void mpu6050_command(uint8_t reg, uint8_t val) {
    uint8_t buf[] = {reg, val};
    i2c_write_blocking(I2C_INSTANCE, MPU6050_ADDR, buf, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buf[6];

    // Read accelerometer
    uint8_t reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buf, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    }

    // Read gyroscope
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buf, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    }

    // Read temperature
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);

    *temp = buf[0] << 8 | buf[1];
}

void mpu6050_convert_raw_accel(uint16_t raw[3], float out[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = (ACCEL_POS_RANGE * 2.0 * ((int16_t)(raw[i])) / ((1<<16)-1))
            * GS_TO_MSS;
    }
}
