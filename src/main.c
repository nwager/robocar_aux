#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#define I2C_INSTANCE i2c_default
#include "mpu6050_utils.h"

#define PWM_MAX_COUNT ((1<<16) - 1)

#define I2C0_SDA_GPIO 8
#define I2C0_SCL_GPIO 9

#define LED_GPIO 25
#define LED_PWM_LOW 0
#define LED_PWM_HIGH (PWM_MAX_COUNT / 20)

#define GS_TO_MSS 9.8

static void mpu6050_reset();

int main() {
    // usb init
    stdio_init_all();
    
    // led pwm init
    const int LED_SLICE = pwm_gpio_to_slice_num(LED_GPIO);
    const int LED_CHAN = pwm_gpio_to_channel(LED_GPIO);
    gpio_set_function(LED_GPIO, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_gpio_to_slice_num(LED_GPIO), PWM_MAX_COUNT);
    pwm_set_enabled(LED_SLICE, true);

    // i2c init
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);
    bi_decl(bi_2pins_with_func(
        I2C0_SDA_GPIO, I2C0_SCL_GPIO, GPIO_FUNC_I2C
    ));

    mpu6050_reset();

    bool led_state = false;

    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    float accel[3];

    while (1) {
        pwm_set_chan_level(LED_SLICE, LED_CHAN, led_state ? LED_PWM_HIGH : LED_PWM_LOW);

        mpu6050_read_raw(accel_raw, gyro_raw, &temp_raw);
        mpu6050_convert_raw_accel(accel_raw, accel);

        printf("Acc. X = %f, Y = %f, Z = %f\n", accel[0], accel[1], accel[2]);

        led_state = !led_state;
        sleep_ms(100);
    }
}

static void mpu6050_reset() {
    mpu6050_command(0x6B, 0x00); // turn on
    mpu6050_command(0x1C, 0x02 << 3); // set accel to +/- 8g
    mpu6050_command(0x1A, 0x02 << 0); // set DLPF to 260hz
}
