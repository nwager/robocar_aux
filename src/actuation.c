#include <limits.h> // for ULONG_MAX

#include <FreeRTOS.h>
#include <task.h>

#include "hardware/pwm.h"

#include "type_utils.h"
#include "task_names.h"
#include "math_utils.h"

#include "actuation.h"

#include <stdio.h>

#define MS_TO_S(t) (((float)(t)) / 1000.0)

#define ACTU_PWM_MASK (((uint32_t)ULONG_MAX) >> ACTU_DEVICE_ID_SHIFT)

/**
 * pwm config settings calculated using formula from
 * https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=2
 * where f_c = 125MHz, f_pwm = 500Hz
 */
#define PWM_CDIV_INT  3
#define PWM_CDIV_FRAC 14
#define PWM_WRAP 64516
#define PWM_PERIOD_US 2000

#define NUM_PWM 2
#define ESC_GPIO 19
#define SERVO_GPIO 18
static const uint8_t const pwm_gpios[NUM_PWM] = { ESC_GPIO, SERVO_GPIO };

/**
 * @brief Proportional error coefficient.
 * 
 */
static const float K_P = 25.0;
/**
 * @brief Integral error coefficient.
 * 
 */
static const float K_I = 1.0;
/**
 * @brief Derivative error coefficient.
 * 
 */
static const float K_D = 5.0;

/**
 * @brief Frequency of velocity PID updates in Hz.
 * 
 */
static const uint PID_HZ = 10;

/**
 * @brief Frequency of steering angle updates in Hz.
 * 
 */
static const uint STEERING_HZ = 10;

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

void pid_vel_task(void *p) {

    TickType_t start_time = xTaskGetTickCount();

    pid_vel_task_arg_t *arg = p;
    TaskHandle_t actu_task_handle = xTaskGetHandle(ACTUATION_TASK_NAME);

    control_msg_t curr_control;
    velocity_stamped_t curr_vel_stamp;

    float err;
    float prev_err = 0.0;

    float p_err;
    float i_err = 0.0;
    float d_err;

    uint output;

    const float dt = 1.0 / PID_HZ;

    while (1) {
        xTaskDelayUntil(&start_time, pdMS_TO_TICKS(1000 / PID_HZ));

        if (xQueuePeek(arg->control_queue, &curr_control, 0) == pdFALSE)
            continue;
        if (xQueuePeek(arg->vel_queue, &curr_vel_stamp, 0) == pdFALSE)
            continue;
        
        err = curr_control.vel - curr_vel_stamp.vel;

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
        actuate_device(DEVICE_ID_ESC, output, 0);

        prev_err = err;
    }
}

void steering_task(void *p) {

    TickType_t start_time = xTaskGetTickCount();

    steering_task_arg_t *arg = p;
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
    for (int i = 0; i < NUM_PWM; i++) {
        uint slice = pwm_gpio_to_slice_num(pwm_gpios[i]);
        // set pwm frequency to 500 hz
        gpio_set_function(pwm_gpios[i], GPIO_FUNC_PWM);
        pwm_set_clkdiv_int_frac(slice, PWM_CDIV_INT, PWM_CDIV_FRAC);
        pwm_set_wrap(slice, PWM_WRAP);
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
    pwm_set_gpio_level(gpio, ((uint32_t)us * PWM_WRAP) / PWM_PERIOD_US);
}
