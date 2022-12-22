#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "freertos_hooks.h"
#include "type_utils.h"
#include "vec3.h"
#include "velocity_handler.h"
#include "task_names.h"
#include "mpu6050.h"
#include "actuation.h"
#include "math_utils.h"
#include "brain_interface.h"

#define BRAIN_CONTROL_MSG_CODE 0xFF

#define BRAIN_SUCCESS 0x00
#define BRAIN_ERROR   0xFF

#define I2C_SDA_GPIO 8
#define I2C_SCL_GPIO 9
#define I2C_BAUD_RATE (400 * 1000)

/**
 * @brief Initializes all queues.
 * 
 * @param control_queue Pointer to control queue handle to set.
 * @param accel_queue Pointer to acceleration queue handle to set.
 * @param vel_queue Pointer to velocity queue handle to set.
 * @param act_queue Pointer to actuation queue handle to set.
 */
static void setup_queues(QueueHandle_t *control_queue,
                         QueueHandle_t *accecl_queue,
                         QueueHandle_t *vel_queue,
                         QueueHandle_t *act_queue);
static void i2c_setup();

#if(configCUSTOM_DEBUG == 1)

volatile unsigned long runtime_stats_counter = 0;
bool runtime_stats_timer_cb(struct repeating_timer *t) {
    runtime_stats_counter++;
    return true;
}

struct repeating_timer timer;
void setup_runtime_stats_timer() {
    add_repeating_timer_us(-500, runtime_stats_timer_cb, NULL, &timer);
}

static char runtime_stats_buf[500];
static char task_list_buf[500];
void print_tasks(void *p) {
    while (1) {
        vTaskGetRunTimeStats(runtime_stats_buf);
        vTaskList(task_list_buf);
        printf("%s\n", runtime_stats_buf);
        printf("%s\n\n", task_list_buf);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

#endif /* configCUSTOM_DEBUG == 1 */

int main() {

    // usb init
    stdio_init_all();

    // i2c init
    i2c_setup();

    QueueHandle_t control_queue, accel_queue, vel_queue, act_queue;
    setup_queues(&control_queue, &accel_queue, &vel_queue, &act_queue);

    SemaphoreHandle_t serial_mutex = xSemaphoreCreateMutex();

#if(configCUSTOM_DEBUG == 1)

    xTaskCreate(
        print_tasks,
        "DEBUG_TASK",
        256,
        NULL,
        tskIDLE_PRIORITY,
        NULL
    );

#endif /* configCUSTOM_DEBUG == 1 */

    process_brain_task_arg_t process_brain_arg = {
        control_queue,
        accel_queue,
        vel_queue,
        act_queue,
        serial_mutex
    };
    xTaskCreate(
        process_brain_task,
        PROCESS_BRAIN_TASK_NAME,
        512,
        &process_brain_arg,
        configMAX_PRIORITIES,
        NULL
    );

    mpu6050_task_arg_t mpu6050_arg = {
        accel_queue
    };
    xTaskCreate(
        mpu6050_task,
        MPU6050_TASK_NAME,
        512,
        &mpu6050_arg,
        tskIDLE_PRIORITY,
        NULL
    );

    update_vel_task_arg_t update_vel_arg = {
        vel_queue,
        accel_queue
    };
    xTaskCreate(
        update_vel_task,
        UPDATE_VEL_TASK_NAME,
        512,
        &update_vel_arg,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    pid_vel_task_arg_t pid_vel_arg = {
        control_queue,
        vel_queue,
        act_queue
    };
    xTaskCreate(
        pid_vel_task,
        PID_VEL_TASK_NAME,
        512,
        &pid_vel_arg,
        configMAX_PRIORITIES - 1,
        NULL
    );

    actuation_task_arg_t actuation_arg = {
        act_queue
    };
    xTaskCreate(
        actuation_task,
        ACTUATION_TASK_NAME,
        512,
        &actuation_arg,
        configMAX_PRIORITIES - 1,
        NULL
    );

    vTaskStartScheduler();

    while (1); // should never reach here
}

static void setup_queues(QueueHandle_t *control_queue,
                         QueueHandle_t *accel_queue,
                         QueueHandle_t *vel_queue,
                         QueueHandle_t *act_queue) {

    *control_queue = xQueueCreate(1, sizeof(control_msg_t));
    control_msg_t zero_control = { 0.0, 0.0 };
    xQueueSendToBack(*control_queue, &zero_control, 0);
    
    *accel_queue = xQueueCreate(1, sizeof(accel_stamped_t));
    accel_stamped_t zero_accel = { nil_time, {0, 0, 0} };
    xQueueSendToBack(*accel_queue, &zero_accel, 0);

    *vel_queue = xQueueCreate(1, sizeof(velocity_stamped_t));
    velocity_stamped_t zero_vel = { nil_time, 0 };
    xQueueSendToBack(*vel_queue, &zero_vel, 0);

    *act_queue = xQueueCreate(4, sizeof(actuation_item_t));
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
