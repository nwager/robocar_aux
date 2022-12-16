#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "mpu6050.h"
#include "type_utils.h"
#include "vec.h"
#include "freertos_hooks.h"

#define MASTER_CONTROL_MSG_CODE 0xFF

#define I2C_SDA_GPIO 8
#define I2C_SCL_GPIO 9
#define I2C_BAUD_RATE (400 * 1000)

typedef struct {
    control_msg_t *control_state;
    QueueHandle_t accel_queue;
} process_master_task_arg_t;
static void process_master_task(void *p);

static int read_from_master(uint8_t buf[], uint n_bytes);
static void send_to_master(uint8_t buf[], uint n_bytes);
static inline void clear_master_buffer();
static void send_error_master(uint n_bytes);
static void i2c_setup();

static TickType_t start_tick;

int main() {
    // usb init
    stdio_init_all();

    // i2c init
    i2c_setup();

    control_msg_t control_state = { 0.0, 0.0 };

    QueueHandle_t accel_queue = xQueueCreate(1, sizeof(vec3_t));

    process_master_task_arg_t process_master_arg = { &control_state, accel_queue };
    xTaskCreate(
        process_master_task,
        "Process_Master_Task",
        128,
        &process_master_arg,
        configMAX_PRIORITIES,
        NULL
    );

    mpu6050_task_arg_t mpu6050_arg = { &control_state, accel_queue };
    xTaskCreate(
        mpu6050_task,
        "MPU6050_Task",
        128,
        &mpu6050_arg,
        tskIDLE_PRIORITY,
        NULL
    );

    vTaskStartScheduler();

    while (1); // should never reach here
}

void process_master_task(void *p) {
    TickType_t start_time = xTaskGetTickCount();
    
    process_master_task_arg_t *arg = p;

    while (1) {
        vTaskDelayUntil(&start_time, pdMS_TO_TICKS(100));

        int cmd = getchar_timeout_us(0);
        if (cmd == PICO_ERROR_TIMEOUT)
            continue; // no cmd, reset
        // master should specify how many bytes are sent or expected
        int payload_size = getchar_timeout_us(0);
        if (payload_size == PICO_ERROR_TIMEOUT)
            continue; // no payload size, reset
        
        if (cmd == MASTER_CONTROL_MSG_CODE) {
            // receive control message
            if (payload_size != sizeof(control_msg_t)) {
                // payload size mismatch, reset
                clear_master_buffer();
                send_error_master(payload_size);
                continue;
            }
            control_msg_t msg;
            read_from_master((uint8_t*)&msg, payload_size);
            // success
            putchar_raw(0x00);
            continue;
        }

        // shouldn't receive data
        if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            clear_master_buffer();
            send_error_master(payload_size);
            continue;
        }
        
        if (cmd == 0xAC) {
            if (payload_size != sizeof(vec3_t)) {
                send_error_master(payload_size);
                continue;
            }
            vec3_t accel;
            xQueueReceive(arg->accel_queue, &accel, pdMS_TO_TICKS(50));
            send_to_master((uint8_t*)&accel, payload_size);
            continue;
        }

        // not a control msg, master requests control data
        if (payload_size == sizeof(control_msg_t)) {
            send_to_master((uint8_t*)(arg->control_state), payload_size);
        } else {
            send_error_master(payload_size);
        }
    } // while (1)
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

static void send_to_master(uint8_t buf[], uint n_bytes) {
    for (int i = 0; i < n_bytes; i++) {
        putchar_raw(buf[i]);
    }
}

static inline void clear_master_buffer() {
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}
}

static void send_error_master(uint n_bytes) {
    for (int i = 0; i < n_bytes; i++) {
        putchar_raw(0xFF);
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
