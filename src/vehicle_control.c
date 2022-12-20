#include <limits.h> // for ULONG_MAX

#include <FreeRTOS.h>
#include <task.h>

#include "hardware/pwm.h"

#include "type_utils.h"
#include "task_names.h"
#include "math_utils.h"

#include "vehicle_control.h"

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
#define MOTOR_GPIO 19
#define SERVO_GPIO 18
static uint8_t pwm_gpios[NUM_PWM] = { MOTOR_GPIO, SERVO_GPIO };

/**
 * @brief Proportional error coefficient.
 * 
 */
static const float kp = 1.0;
/**
 * @brief Integral error coefficient.
 * 
 */
static const float ki = 1.0;
/**
 * @brief Derivative error coefficient.
 * 
 */
static const float kd = 1.0;

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

    uint16_t output;

    const float dt = 0.02;

    while (1) {
        xTaskDelayUntil(&start_time, pdMS_TO_TICKS((int)(1000.0 * dt)));

        if (xQueuePeek(arg->control_queue, &curr_control, 0) == pdFALSE)
            continue;
        if (xQueuePeek(arg->vel_queue, &curr_vel_stamp, 0) == pdFALSE)
            continue;

        err = curr_control.vel - curr_vel_stamp.vel;

        p_err = err;
        i_err += err * dt;
        d_err = (err - prev_err) / dt;

        output = clampi(
            PWM_ZERO + ((kp * p_err) + (ki * i_err) + (kd * d_err)),
            PWM_MIN,
            PWM_MAX
        );

        xTaskNotify(
            actu_task_handle,
            format_actuation_notif(DEVICE_ESC, output),
            eSetBits
        );

        prev_err = err;
    }
}

void actuation_task(void *p) {
    (void)p;

    // set up PWM
    pwm_init_devices();

    actu_device_id_t device_id;
    uint16_t pwm_micros;
    uint32_t notif_val;

    while (1) {
        if (xTaskNotifyWait(0, ULONG_MAX, &notif_val, 0) == pdFALSE)
            continue;
        
        device_id = notif_val >> ACTU_DEVICE_ID_SHIFT;
        pwm_micros = notif_val & ACTU_PWM_MASK;

        switch (device_id) {
            case DEVICE_ESC:
                // actuate esc
                break;
            case DEVICE_SERVO:
                // actuate servo
                break;
        }
    }
}

uint32_t format_actuation_notif(actu_device_id_t id, uint pwm) {
    return (id << ACTU_DEVICE_ID_SHIFT) | (pwm & ACTU_PWM_MASK);
}

static void pwm_init_devices() {
    for (int i = 0; i < NUM_PWM; i++) {
        uint slice = pwm_gpio_to_slice_num(pwm_gpios[i]);
        // set pwm frequency to 500 hz
        gpio_set_function(pwm_gpios[i], GPIO_FUNC_PWM);
        pwm_set_clkdiv_int_frac(slice, PWM_CDIV_INT, PWM_CDIV_FRAC);
        pwm_set_wrap(slice, PWM_WRAP);
        pwm_set_enabled(slice, true);
        // start at neutral
        pwm_write_us(pwm_gpios[i], PWM_ZERO);
    }
}

static inline void pwm_write_us(uint gpio, uint us) {
    pwm_set_gpio_level(gpio, ((uint32_t)us * PWM_WRAP) / PWM_PERIOD_US);
}
