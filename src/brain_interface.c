#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "task_names.h"
#include "type_utils.h"
#include "actuation.h"

#include "brain_interface.h"

#define BRAIN_CONTROL_MSG_CODE 0xFF

#define BRAIN_SUCCESS 0x00
#define BRAIN_ERROR   0xFF

typedef struct {
    uint8_t command;
    uint8_t payload_size;
} msg_header_t;

static int read_from_brain(uint8_t buf[], uint n_bytes);
static void send_to_brain(uint8_t buf[], uint n_bytes);
static inline void clear_brain_buffer();
static void send_error_brain(uint n_bytes);
static inline bool read_brain_header(msg_header_t *dst);
static inline void acknowledge_brain(bool ack);

static SemaphoreHandle_t serial_mutex;

void process_brain_task(void *p) {
    TickType_t start_time = xTaskGetTickCount();
    
    process_brain_task_arg_t *arg = p;
    serial_mutex = arg->serial_mutex;

    TaskHandle_t mpu6050_task_handle = xTaskGetHandle(MPU6050_TASK_NAME);
    TaskHandle_t actu_task_handle = xTaskGetHandle(ACTUATION_TASK_NAME);

    while (1) {
        vTaskDelayUntil(&start_time, pdMS_TO_TICKS(100));

        msg_header_t hdr;
        if (!read_brain_header(&hdr))
            continue; // if fail, we know buffer is empty and no payload
        
        if (hdr.command == BRAIN_CONTROL_MSG_CODE) {
            // receive control message
            if (hdr.payload_size != sizeof(control_msg_t)) {
                // payload size mismatch, reset
                clear_brain_buffer();
                acknowledge_brain(false);
                continue;
            }
            control_msg_t control_msg;
            read_from_brain((uint8_t*)&control_msg, hdr.payload_size);

            xQueueOverwrite(arg->control_queue, &control_msg);
            // actuate servo
            actuation_item_t servo_item = {
                angle_to_pwm(control_msg.steer),
                DEVICE_ID_SERVO
            };
            bool actuation_status = actuate_device(
                DEVICE_ID_SERVO, angle_to_pwm(control_msg.steer), 2
            );
            acknowledge_brain(actuation_status);
            continue;
        }

        // reset mpu6050
        if (hdr.command == 0xAA) {
            if (hdr.payload_size != 0) {
                send_error_brain(hdr.payload_size);
                continue;
            }
            xTaskNotifyGive(mpu6050_task_handle);
            acknowledge_brain(true);
            continue;
        }

        // shouldn't receive data so buffer should be empty
        if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            clear_brain_buffer();
            send_error_brain(hdr.payload_size);
            continue;
        }
        
        if (hdr.command == 0xAC) {
            if (hdr.payload_size != sizeof(vec3_t)) {
                send_error_brain(hdr.payload_size);
                continue;
            }
            accel_stamped_t accel;
            BaseType_t status = xQueuePeek(
                arg->accel_queue, &accel, pdMS_TO_TICKS(50)
            );
            if (status == pdTRUE) {
                send_to_brain((uint8_t*)&(accel.accel), hdr.payload_size);
            } else {
                send_error_brain(hdr.payload_size);
            }
            continue;
        }

        if (hdr.command == 0x10) {
            if (hdr.payload_size == sizeof(float)) {
                velocity_stamped_t tmp;
                if (xQueuePeek(arg->vel_queue, &tmp, 0) == pdTRUE) {
                    send_to_brain((uint8_t*)&(tmp.vel), hdr.payload_size);
                } else {
                    send_error_brain(hdr.payload_size);
                }
            } else {
                send_error_brain(hdr.payload_size);
            }
            continue;
        }

        // not a control msg, brain requests control data
        if (hdr.payload_size == sizeof(control_msg_t)) {
            control_msg_t ctrl_state;
            if (xQueuePeek(arg->control_queue, &ctrl_state, 0) == pdTRUE) {
                send_to_brain((uint8_t*)(&ctrl_state), hdr.payload_size);
            } else {
                send_error_brain(hdr.payload_size);
            }
        } else {
            send_error_brain(hdr.payload_size);
        }
    } // while (1)
}

static int read_from_brain(uint8_t buf[], uint n_bytes) {
    xSemaphoreTake(serial_mutex, portMAX_DELAY);
    int i = 0;
    for (i = 0; i < n_bytes; i++) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT)
            break;
        buf[i] = (uint8_t)c;
    }
    xSemaphoreGive(serial_mutex);
    return i;
}

static void send_to_brain(uint8_t buf[], uint n_bytes) {
    xSemaphoreTake(serial_mutex, portMAX_DELAY);
    for (int i = 0; i < n_bytes; i++) {
        putchar_raw(buf[i]);
    }
    xSemaphoreGive(serial_mutex);
}

static inline void clear_brain_buffer() {
    xSemaphoreTake(serial_mutex, portMAX_DELAY);
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    xSemaphoreGive(serial_mutex);
}

static void send_error_brain(uint n_bytes) {
    char error = BRAIN_ERROR;
    // loop instead of allocating array so we don't need to
    // dynamically allocate lots of stack space
    for (int i = 0; i < n_bytes; i++) {
        send_to_brain(&error, 1);
    }
}

static inline bool read_brain_header(msg_header_t *dst) {
    return read_from_brain(
            (uint8_t*)dst, sizeof(msg_header_t)
        ) == sizeof(msg_header_t);
}

static void acknowledge_brain(bool ack) {
    char ack_byte = ack ? BRAIN_SUCCESS : BRAIN_ERROR;
    send_to_brain(&ack_byte, 1);
}