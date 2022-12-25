#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "pico/stdlib.h"

#include "task_names.h"
#include "type_utils.h"
#include "actuation.h"
#include "mpu6050.h"
#include "vec3.h"

#include "brain_interface.h"

#define BRAIN_SUCCESS 0x00
#define BRAIN_ERROR   0xFF

typedef struct {
    uint8_t command;
    uint8_t payload_size;
} msg_header_t;
// high means brain is sending data and you should acknowledge,
// low means brain is requesting data so acknowlege not necessary
#define SHOULD_ACK_MASK (1 << 7)

typedef enum {
    // dir bit low, brain requesting data
    CMD_GET_ACCEL = 0,
    CMD_GET_CONTROL,
    CMD_CALIBRATE_ACCEL,
    // dir bit high, brain sending data
    CMD_RESET = (0 | SHOULD_ACK_MASK),
    CMD_SET_CONTROL,
    CMD_SET_ACCEL_OFFSETS,
} brain_command_code_t;

static int read_from_brain(uint8_t buf[], uint n_bytes);
static void send_to_brain(uint8_t buf[], uint n_bytes);
static inline void clear_brain_buffer();
static void send_error_to_brain(uint n_bytes);
static inline bool read_brain_header(msg_header_t *dst);
static inline void acknowledge_brain(bool ack);

static bool handle_cmd_reset();

static bool handle_cmd_get_control(msg_header_t *hdr,
                                   process_brain_task_arg_t *arg);

static bool handle_cmd_set_control(msg_header_t *hdr,
                                   process_brain_task_arg_t *arg);

static bool handle_cmd_get_accel(msg_header_t *hdr,
                                 process_brain_task_arg_t *arg);

static bool handle_cmd_calibrate_accel(msg_header_t *hdr);

static bool handle_cmd_set_accel_offsets(msg_header_t *hdr);

static SemaphoreHandle_t serial_mutex;
static TaskHandle_t mpu6050_task_handle;
static TaskHandle_t actu_task_handle;
static TaskHandle_t pid_vel_control_task_handle;

void process_brain_task(void *p) {
    TickType_t start_time = xTaskGetTickCount();
    
    process_brain_task_arg_t *arg = p;
    serial_mutex = arg->serial_mutex;

    mpu6050_task_handle = xTaskGetHandle(MPU6050_TASK_NAME);
    actu_task_handle = xTaskGetHandle(ACTUATION_TASK_NAME);
    pid_vel_control_task_handle = xTaskGetHandle(PID_VEL_CONTROL_TASK_NAME);

    while (1) {
        vTaskDelayUntil(&start_time, pdMS_TO_TICKS(100));

        msg_header_t hdr;
        if (!read_brain_header(&hdr))
            continue; // if fail, we know buffer is empty and no payload

        bool result = false;
        switch (hdr.command) {
            case CMD_RESET:
                result = handle_cmd_reset();
                break;
            case CMD_SET_CONTROL:
                result = handle_cmd_set_control(&hdr, arg);
                break;
            case CMD_GET_CONTROL:
                result = handle_cmd_get_control(&hdr, arg);
                break;
            case CMD_GET_ACCEL:
                result = handle_cmd_get_accel(&hdr, arg);
                break;
            case CMD_CALIBRATE_ACCEL:
                result = handle_cmd_calibrate_accel(&hdr);
                break;
            case CMD_SET_ACCEL_OFFSETS:
                result = handle_cmd_set_accel_offsets(&hdr);
                break;
        }

        bool should_ack = hdr.command & SHOULD_ACK_MASK;

        if (result) {
            if (should_ack)
                acknowledge_brain(true);
            continue;
        }

        // error occurred, choose the correct response based on the command
        clear_brain_buffer();
        if (should_ack) {
            acknowledge_brain(false);
        } else {
            send_error_to_brain(hdr.payload_size);
        }
    } // while (1)
}

// Command handlers

static bool handle_cmd_set_control(msg_header_t *hdr,
                                   process_brain_task_arg_t *arg) {

    if (hdr->payload_size != sizeof(control_msg_t)) {
        // payload size mismatch
        return false;
    }
    control_msg_t control_msg;
    if (read_from_brain((uint8_t*)&control_msg, hdr->payload_size)
        != hdr->payload_size) {
        return false;
    }
    // update controls
    xQueueOverwrite(arg->control_queue, &control_msg);
    return true;
}

static bool handle_cmd_get_control(msg_header_t *hdr,
                                   process_brain_task_arg_t *arg) {
    uint8_t payload_size = hdr->payload_size;
    if (payload_size == sizeof(control_msg_t)) {
        // get current steer - same as target servo position
        control_msg_t control_state, msg;
        if (xQueuePeek(arg->control_queue, &control_state, 0) == pdFALSE)
            return false;
        
        // get current velocity
        velocity_stamped_t vel_state;
        if (xQueuePeek(arg->vel_queue, &vel_state, 0) == pdFALSE)
            return false;

        msg.steer = control_state.steer;
        msg.vel = vel_state.vel;
        
        send_to_brain((uint8_t*)(&msg), payload_size);
        return true;
    }
    return false;
}

static bool handle_cmd_reset() {
    clear_brain_buffer();
    bool res = true;
    res = res && actuate_device(DEVICE_ID_SERVO, angle_to_pwm(0), 0);
    res = res && actuate_device(DEVICE_ID_ESC, ESC_PWM_ZERO, 0);
    xTaskNotifyGive(mpu6050_task_handle); // reset mpu6050
    xTaskNotifyGive(pid_vel_control_task_handle); // reset pid i-error
    vTaskDelay(pdMS_TO_TICKS(100)); // give time to reset
    return res;
}

static bool handle_cmd_get_accel(msg_header_t *hdr,
                                 process_brain_task_arg_t *arg) {
    uint8_t payload_size = hdr->payload_size;
    if (payload_size != sizeof(vec3_t)) {
        return false;
    }
    accel_stamped_t acc_stmp;
    if (xQueuePeek(arg->accel_queue, &acc_stmp, pdMS_TO_TICKS(50)) == pdTRUE) {
        send_to_brain((uint8_t*)&(acc_stmp.accel), payload_size);
        return true;
    }
    return false;
}

static bool handle_cmd_calibrate_accel(msg_header_t *hdr) {

    if (hdr->payload_size == 0)
        return mpu6050_calibrate(NULL, NULL);

    vec3_t grav_offset, fwd_dir;

    if (!mpu6050_calibrate(&grav_offset, &fwd_dir))
        return false;
    send_to_brain((uint8_t*)&grav_offset, sizeof(grav_offset));
    send_to_brain((uint8_t*)&fwd_dir, sizeof(fwd_dir));
    return true;
}

static bool handle_cmd_set_accel_offsets(msg_header_t *hdr) {
    uint payload_size = hdr->payload_size;
    if (payload_size != 2*sizeof(vec3_t)) {
        return false;
    }
    
    vec3_t buf[2];
    if (read_from_brain((uint8_t*)buf, payload_size) != payload_size)
        return false;

    mpu6050_set_offsets(buf + 0, buf + 1);
    return true;
}

// Serial communication helpers

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

static void send_error_to_brain(uint n_bytes) {
    char error = BRAIN_ERROR;
    // loop instead of allocating array so we don't need to
    // dynamically allocate lots of stack space
    for (int i = 0; i < n_bytes; i++) {
        send_to_brain(&error, 1);
    }
}

static inline bool read_brain_header(msg_header_t *dst) {
    return read_from_brain((uint8_t*)dst, sizeof(msg_header_t))
        == sizeof(msg_header_t);
}

static void acknowledge_brain(bool ack) {
    char ack_byte = ack ? BRAIN_SUCCESS : BRAIN_ERROR;
    send_to_brain(&ack_byte, 1);
}