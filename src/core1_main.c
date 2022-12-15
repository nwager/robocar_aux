#include "core1_main.h"

#define WHEEL_REV_GPIO 15

static volatile bool new_rev_reading = false;
static volatile absolute_time_t prev_rev, curr_rev;

extern queue_t command_queue;
extern queue_t response_queue;

void core1_entry() {
    i2c_setup();
    mpu6050_reset();

    prev_rev = get_absolute_time();

    gpio_pull_up(WHEEL_REV_GPIO);
    gpio_set_irq_enabled_with_callback(
        WHEEL_REV_GPIO,
        GPIO_IRQ_EDGE_FALL,
        true,
        &wheel_rev_irq_handler
    );

    // "globals" to store measurements on stack and copies for queue
    float a[3], a_cpy[3];
    float speed = 0.0, speed_cpy;
    umicros_t rev_dt;

    core_command_t cmd;
    core_response_t res;

    while (1) {
        // if new wheel revolution reading, get speed
        if (new_rev_reading) {
            new_rev_reading = false;

            uint32_t i_status = save_and_disable_interrupts();
            rev_dt = absolute_time_diff_us(prev_rev, curr_rev);
            restore_interrupts(i_status);

            speed = calculate_speed(rev_dt);
        }
        // get acceleration
        mpu6050_get_accel(a);

        // check for commands and process if necessary
        res.code = FAIL; // default to fail
        res.payload_size = 0; // default to no data
        if (queue_try_remove(&command_queue, &cmd)) {
            // map command code to action
            switch (cmd) {
                case NOOP:
                    res.code = SUCCESS;
                    break;
                case CALIBRATE_ACCEL:
                    mpu6050_calibrate();
                    res.code = SUCCESS;
                    break;
                case GET_SPEED:
                    speed_cpy = speed;
                    // FOR TESTING
                    speed_cpy = 123.125;
                    res.payload = &speed_cpy;
                    res.payload_size = sizeof(speed_cpy);
                    res.code = SUCCESS;
                    break;
                case GET_ACCEL:
                    vec_copy(a, a_cpy);
                    res.payload = (void*)a_cpy;
                    res.payload_size = sizeof(a_cpy);
                    res.code = SUCCESS;
                    break;
                case RESET_MPU6050:
                    mpu6050_reset();
                    res.code = SUCCESS;
                    break;
            }
            // send response
            queue_add_blocking(&response_queue, &res);
        }

        sleep_ms(20);
    }
}

static void i2c_setup() {
    i2c_init(I2C_INSTANCE, I2C_BAUD_RATE);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    bi_decl(bi_2pins_with_func(
        I2C_SDA_GPIO, I2C_SCL_GPIO, GPIO_FUNC_I2C
    ));
}

static void mpu6050_reset() {
    mpu6050_init();
    mpu6050_set_dlpf(MPU6050_DLPF_260HZ);
    mpu6050_set_range(MPU6050_ACCEL_RANGE_8G);
}

static void wheel_rev_irq_handler(uint _gpio, uint32_t _event_mask) {
    new_rev_reading = true;
    prev_rev = curr_rev;
    curr_rev = get_absolute_time();
}

static inline float calculate_speed(umicros_t dt) {
    return WHEEL_C_M * 1000000.0 / dt;
}
