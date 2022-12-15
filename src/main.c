#include <stdio.h>
#include <math.h> // for NAN
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/pwm.h"

#include "core1_main.h"
#include "type_utils.h"
#include "core_comm_queue.h"
#include "vec.h"

#define MASTER_CONTROL_MSG_CODE 0xFF
typedef struct { float vel, heading; } control_msg_t;

#define LED_GPIO 25

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

queue_t command_queue;
core_command_t cmd;
queue_t response_queue;
core_response_t res;

static float target_speed = 0.0;
static float target_heading = 0.0;

static bool process_master_cmd(uint8_t cmd);
static int read_from_master(uint8_t buf[], uint n_bytes);
static int send_to_master(uint8_t buf[], uint n_bytes);
static inline void clear_master_buffer();
static void pwm_device_init();
static inline void pwm_write_us(uint8_t pwm_idx, uint us);
static void pid_update();

int main() {
    // usb init
    stdio_init_all();
    
    sleep_ms(2000);
    printf("Starting up...\n");

    // blink led init
    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);

    // intercore communication queue init
    queue_init(&command_queue, sizeof(core_command_t), 1);
    queue_init(&response_queue, sizeof(core_response_t), 1);

    // set up pwm control
    pwm_device_init();

    // start core 1
    multicore_launch_core1(core1_entry);

    while (1) {
        // check for command from computer
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            bool status = process_master_cmd(c);
            if (status){
                // TODO: command succeeded
            } else {
                // TODO: command failed
                clear_master_buffer();
            }
            gpio_put(LED_GPIO, !status); // on iff fail
        }
    }
}

static bool process_master_cmd(uint8_t cmd) {
    // master should specify how many bytes are sent or expected
    int payload_size = getchar_timeout_us(0);
    if (payload_size == PICO_ERROR_TIMEOUT)
        return false;
    
    if (cmd == MASTER_CONTROL_MSG_CODE) {
        // receive control message
        if (payload_size != sizeof(control_msg_t)) {
            return false;
        }
        // if not a control message, it's a request for data
        control_msg_t msg;
        read_from_master((uint8_t*)&msg, payload_size);
        send_to_master((uint8_t*)&msg, payload_size);
        return true;
    }
    // get control data from core 1
    queue_add_blocking(&command_queue, &cmd);
    queue_remove_blocking(&response_queue, &res);
    // if payload sizes don't match, or error, send all NaNs
    if (payload_size != res.payload_size || res.code == FAIL) {
        for (int i = 0; i < payload_size; i++) {
            float val = NAN;
            send_to_master((uint8_t*)&val, sizeof(val));
        }
        return false;
    }
    // send data to master
    send_to_master(res.payload, payload_size);
    return true;
}

static int read_from_master(uint8_t buf[], uint n_bytes) {
    for (int i = 0; i < n_bytes; i++) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT)
            return i;
        buf[i] = (uint8_t)c;
    }
    return n_bytes;
}

static int send_to_master(uint8_t buf[], uint n_bytes) {
    for (int i = 0; i < n_bytes; i++) {
        putchar_raw(buf[i]);
    }
}

static inline void clear_master_buffer() {
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}
}

static void pwm_device_init() {
    for (int i = 0; i < NUM_PWM; i++) {
        uint slice = pwm_gpio_to_slice_num(pwm_gpios[i]);
        // set pwm frequency to 500 hz
        gpio_set_function(pwm_gpios[i], GPIO_FUNC_PWM);
        pwm_set_clkdiv_int_frac(slice, PWM_CDIV_INT, PWM_CDIV_FRAC);
        pwm_set_wrap(slice, PWM_WRAP);
        pwm_set_enabled(slice, true);
        // start at neutral
        pwm_write_us(pwm_gpios[i], 1500);
    }
}

static inline void pwm_write_us(uint8_t gpio, uint us) {
    pwm_set_gpio_level(gpio, ((uint32_t)us * PWM_WRAP) / PWM_PERIOD_US);
}
