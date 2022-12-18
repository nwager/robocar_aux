#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "type_utils.h"
#include "vec3.h"
#include "velocity_handler.h"
#include "task_names.h"
#include "mpu6050.h"
#include "freertos_hooks.h"

#define MASTER_CONTROL_MSG_CODE 0xFF

#define I2C_SDA_GPIO 8
#define I2C_SCL_GPIO 9
#define I2C_BAUD_RATE (400 * 1000)

typedef struct {
    uint8_t command;
    uint8_t payload_size;
} msg_header_t;

typedef struct {
    control_msg_t *control_state;
    QueueHandle_t accel_queue;
    QueueHandle_t vel_queue;
} process_master_task_arg_t;
static void process_master_task(void *p);

static int read_from_master(uint8_t buf[], uint n_bytes);
static void send_to_master(uint8_t buf[], uint n_bytes);
static inline void clear_master_buffer();
static void send_error_master(uint n_bytes);
static inline bool read_master_header(msg_header_t *dst);

/**
 * @brief Sets up acceleration and velocity queues and initializes them
 *     with 0-values.
 * 
 * @param acc Pointer to acceleration queue handle to set.
 * @param vel Pointer to velocity queue handle to set.
 */
static void setup_queues(QueueHandle_t *accq, QueueHandle_t *velq);
static void i2c_setup();

int main() {

    // usb init
    stdio_init_all();

    // i2c init
    i2c_setup();

    control_msg_t control_state = { 0.0, 0.0 };

    QueueHandle_t accel_queue, vel_queue;
    setup_queues(&accel_queue, &vel_queue);

    process_master_task_arg_t process_master_arg = {
        &control_state,
        accel_queue,
        vel_queue
    };
    xTaskCreate(
        process_master_task,
        PROCESS_MASTER_TASK_NAME,
        512,
        &process_master_arg,
        configMAX_PRIORITIES,
        NULL
    );

    mpu6050_task_arg_t mpu6050_arg = { &control_state, accel_queue };
    xTaskCreate(
        mpu6050_task,
        MPU6050_TASK_NAME,
        256,
        &mpu6050_arg,
        tskIDLE_PRIORITY,
        NULL
    );

    update_vel_task_arg_t update_vel_arg = { vel_queue, accel_queue };
    xTaskCreate(
        update_vel_task,
        UPDATE_VEL_TASK_NAME,
        512,
        &update_vel_arg,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    vTaskStartScheduler();

    while (1); // should never reach here
}

void process_master_task(void *p) {
    TickType_t start_time = xTaskGetTickCount();
    
    process_master_task_arg_t *arg = p;

    TaskHandle_t mpu6050_task = xTaskGetHandle(MPU6050_TASK_NAME);

    while (1) {
        vTaskDelayUntil(&start_time, pdMS_TO_TICKS(100));

        msg_header_t hdr;
        if (!read_master_header(&hdr))
            continue; // if fail, we know buffer is empty and no payload
        
        if (hdr.command == MASTER_CONTROL_MSG_CODE) {
            // receive control message
            if (hdr.payload_size != sizeof(control_msg_t)) {
                // payload size mismatch, reset
                clear_master_buffer();
                send_error_master(hdr.payload_size);
                continue;
            }
            control_msg_t msg;
            read_from_master((uint8_t*)&msg, hdr.payload_size);
            putchar_raw(0x00); // ack byte
            continue;
        }


        if (hdr.command == 0xAA) {
            if (hdr.payload_size != 0) {
                send_error_master(hdr.payload_size);
                continue;
            }
            xTaskNotifyGive(mpu6050_task);
            putchar_raw(0x00); // ack byte
            continue;
        }

        // shouldn't receive data so buffer should be empty
        if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            clear_master_buffer();
            send_error_master(hdr.payload_size);
            continue;
        }
        
        if (hdr.command == 0xAC) {
            if (hdr.payload_size != sizeof(vec3_t)) {
                send_error_master(hdr.payload_size);
                continue;
            }
            accel_stamped_t accel;
            BaseType_t status = xQueuePeek(
                arg->accel_queue, &accel, pdMS_TO_TICKS(50)
            );
            if (status == pdTRUE) {
                send_to_master((uint8_t*)&(accel.accel), hdr.payload_size);
            } else {
                send_error_master(hdr.payload_size);
            }
            continue;
        }

        if (hdr.command == 0x10) {
            if (hdr.payload_size == sizeof(float)) {
                velocity_stamped_t tmp;
                if (xQueuePeek(arg->vel_queue, &tmp, 0) == pdTRUE) {
                    send_to_master((uint8_t*)&(tmp.vel), hdr.payload_size);
                } else {
                    send_error_master(hdr.payload_size);
                }
            } else {
                send_error_master(hdr.payload_size);
            }
            continue;
        }

        // not a control msg, master requests control data
        if (hdr.payload_size == sizeof(control_msg_t)) {
            send_to_master((uint8_t*)(arg->control_state), hdr.payload_size);
        } else {
            send_error_master(hdr.payload_size);
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
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
}

static void send_error_master(uint n_bytes) {
    for (int i = 0; i < n_bytes; i++) {
        putchar_raw(0xFF);
    }
}

static inline bool read_master_header(msg_header_t *dst) {
    return read_from_master(
            (uint8_t*)dst, sizeof(msg_header_t)
        ) == sizeof(msg_header_t);
}

static void setup_queues(QueueHandle_t *accq, QueueHandle_t *velq) {
    *accq = xQueueCreate(1, sizeof(accel_stamped_t));
    accel_stamped_t zero_a = { nil_time, {0, 0, 0} };
    xQueueSendToBack(*accq, &zero_a, 0);

    *velq = xQueueCreate(1, sizeof(velocity_stamped_t));
    velocity_stamped_t zero_v = { nil_time, 0 };
    xQueueSendToBack(*velq, &zero_v, 0);
}

static void i2c_setup() {
    i2c_init(MPU6050_I2C, I2C_BAUD_RATE);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    bi_decl(bi_2pins_with_func(
        I2C_SDA_GPIO, I2C_SCL_GPIO, GPIO_FUNC_I2C
    ));
}
