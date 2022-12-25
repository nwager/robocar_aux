#include <stdlib.h>
#include <limits.h> // for ULONG_MAX

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "FreeRTOSConfig.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "type_utils.h"
#include "math_utils.h"

#include "actuation.h"

#include <stdio.h>

#define MS_TO_S(t) (((float)(t)) / 1000.0)

#define ACTU_PWM_MASK (((uint32_t)ULONG_MAX) >> ACTU_DEVICE_ID_SHIFT)

#define NUM_PWM 2
#define ESC_GPIO 20
#define SERVO_GPIO 21
static const uint8_t const pwm_gpios[NUM_PWM] = { ESC_GPIO, SERVO_GPIO };

/**
 * @brief Proportional error coefficient.
 */
static const float K_P = 25.0;

/**
 * @brief Integral error coefficient.
 */
static const float K_I = 1.0;

/**
 * @brief Derivative error coefficient.
 */
static const float K_D = 5.0;

/**
 * @brief Frequency of velocity PID updates in Hz.
 */
static const uint PID_HZ = 60;

/**
 * @brief How long to wait between pid integral error resets, in ms.
 */
static const uint I_ERR_RESET_INTERVAL_MS = 3000;

/**
 * @brief Frequency of steering angle updates in Hz.
 */
static const uint STEERING_HZ = 60;

static QueueHandle_t act_queue;
static TaskHandle_t actuation_task_handle;
static bool actuation_initialized = false;

/**
 * @brief Initialize all GPIOs in pwm_gpios as PWM devices.
 * 
 */
static void pwm_init_devices();
/**
 * @brief Write microseconds to the specified PWM GPIO.
 * 
 * @param gpio GPIO to write to.
 * @param micros PWM microseconds (how long the signal is high).
 */
static void pwm_write_us(uint gpio, uint micros);

void pid_vel_control_task(void *p) {

    TickType_t start_time = xTaskGetTickCount();

    pid_vel_control_task_arg_t *arg = p;

    control_msg_t target_control;
    velocity_stamped_t curr_vel_stamp;

    float err;
    float prev_err = 0.0;

    float p_err;
    float i_err = 0.0;
    float d_err;

    uint output;

    const float dt = 1.0 / PID_HZ;

    TickType_t last_i_err_reset = start_time;

    while (1) {
        xTaskDelayUntil(&start_time, pdMS_TO_TICKS(1000 / PID_HZ));

        // check for manual reset
        if (xTaskNotifyWait(0, ULONG_MAX, NULL, 0) == pdTRUE) {
            i_err = 0;
            prev_err = 0;
        }

        // check for i reset timer
        TickType_t i_err_reset_dt = xTaskGetTickCount() - last_i_err_reset;
        if (i_err_reset_dt >= pdMS_TO_TICKS(I_ERR_RESET_INTERVAL_MS)) {
            i_err = 0;
            prev_err = 0;
        }

        if (xQueuePeek(arg->control_queue, &target_control, 0) == pdFALSE)
            continue;
        if (xQueuePeek(arg->vel_queue, &curr_vel_stamp, 0) == pdFALSE)
            continue;
        
        err = target_control.vel - curr_vel_stamp.vel;

        // calculate PID errors
        p_err = err;
        i_err += err * dt;
        d_err = (err - prev_err) / dt;
        // combine errors to get set point
        output = clampi(
            ESC_PWM_ZERO + ((K_P * p_err) + (K_I * i_err) + (K_D * d_err)),
            ESC_PWM_MIN,
            ESC_PWM_MAX
        );

        // if under velocity threshold, just stop
        actuate_device(
            DEVICE_ID_ESC,
            abs(target_control.vel) > 0.1 ? output : ESC_PWM_ZERO,
            0
        );

        prev_err = err;
    }
}

void steer_control_task(void *p) {

    TickType_t start_time = xTaskGetTickCount();

    steer_control_task_arg_t *arg = p;
    control_msg_t control;

    while (1) {
        xTaskDelayUntil(&start_time, pdMS_TO_TICKS(1000 / STEERING_HZ));

        if (xQueuePeek(arg->control_queue, &control, 0) == pdFALSE)
            continue;

        actuate_device(DEVICE_ID_SERVO, angle_to_pwm(control.steer), 0);
    }
}

void actuation_task(void *p) {

    actuation_task_arg_t *arg = p;

    actuation_task_handle = xTaskGetCurrentTaskHandle();
    act_queue = arg->act_queue;

    actuation_initialized = true;

    // set up PWM
    pwm_init_devices();

    actuation_item_t item;

    while (1) {
        if (xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY) == pdFALSE)
            continue;
        
        while (uxQueueMessagesWaiting(arg->act_queue) > 0) {
            if (xQueueReceive(arg->act_queue, &item, 0) == pdFALSE)
                continue;
            
            uint device_gpio;

            switch (item.id) {
                case DEVICE_ID_ESC:
                    device_gpio = ESC_GPIO;
                    break;
                case DEVICE_ID_SERVO:
                    device_gpio = SERVO_GPIO;
                    break;
            }

            pwm_write_us(device_gpio, item.value);
        }
    }
}

static void pwm_init_devices() {
    /*
     * These magic numbers were chosen so 1 tick = 1 us, meaning you can
     * set the level equal to the microseconds you want to write.
     * The wrap is set at 20,000 ticks which is the same as a 20,000 us
     * period (i.e. 50 Hz used by servos and escs). These aren't the
     * optimal settings for maximum PWM resolution, but a resolution
     * of 20,000 is plenty for these devices.
     */
    const uint pwm_wrap = 20000;
    const uint clk_div = 125;

    for (int i = 0; i < NUM_PWM; i++) {
        uint slice = pwm_gpio_to_slice_num(pwm_gpios[i]);
        gpio_set_function(pwm_gpios[i], GPIO_FUNC_PWM);
        pwm_set_clkdiv_int_frac(slice, clk_div, 0);
        pwm_set_wrap(slice, pwm_wrap);
        pwm_set_enabled(slice, true);
    }
    // start at neutral
    pwm_write_us(SERVO_GPIO, SERVO_PWM_ZERO);
    pwm_write_us(ESC_GPIO, ESC_PWM_ZERO);
}

uint angle_to_pwm(float angle) {
    float sig = mapf(
        angle,
        SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
        SERVO_PWM_MIN, SERVO_PWM_MAX
    );
    return clampi(sig, SERVO_PWM_MIN, SERVO_PWM_MAX);
}

bool actuate_device(actuation_device_id_t id,
                    uint value,
                    TickType_t timeout) {

    if (!actuation_initialized)
        return false;

    actuation_item_t item = { value, id };

    BaseType_t status = xQueueSendToBack(act_queue, &item, timeout);
    xTaskNotifyGive(actuation_task_handle); // should always notify

    return status == pdTRUE;
}

static inline void pwm_write_us(uint gpio, uint us) {
    pwm_set_gpio_level(gpio, us);
}
