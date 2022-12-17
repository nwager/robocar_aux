#include "velocity_handler.h"

#define MICROS_TO_S(t) (((float)(t)) / 1000000.0)

static absolute_time_t prev_time, curr_time;
static TaskHandle_t update_vel_task_handle;

static void wheel_rev_isr(uint gpio, uint32_t event_mask) {
    (void)gpio;
    (void)event_mask;

    prev_time = curr_time;
    curr_time = get_absolute_time();
    xTaskNotifyFromISR(
        update_vel_task_handle,
        WHEEL_NOTIF_MASK,
        eSetBits,
        NULL
    );
}

void update_vel_task(void *p) {
    update_vel_task_handle = xTaskGetCurrentTaskHandle();

    curr_time = nil_time;
    prev_time = nil_time;

    velocity_stamped_t calc_vel, tmp_vel;

    accel_stamped_t prev_accel, tmp_accel;
    prev_accel.time = nil_time;

    gpio_pull_up(WHEEL_REV_GPIO);
    gpio_set_irq_enabled_with_callback(
        WHEEL_REV_GPIO,
        GPIO_IRQ_EDGE_FALL,
        true,
        wheel_rev_isr
    );

    update_vel_task_arg_t *arg = p;
    uint32_t notif_val;

    while (1) {
        xTaskNotifyWait(0, ULONG_MAX, &notif_val, portMAX_DELAY);

        if ((notif_val & WHEEL_NOTIF_MASK) && !is_nil_time(prev_time)) {
            // wheel
            umicros_t dt_us = absolute_time_diff_us(prev_time, curr_time);

            calc_vel.vel = WHEEL_C_M / MICROS_TO_S(dt_us);
            calc_vel.time = get_absolute_time();

            xQueueOverwrite(arg->vel_queue, &calc_vel);
        } else if (notif_val & ACCEL_NOTIF_MASK) {
            // accel
            if (xQueuePeek(arg->accel_queue, &tmp_accel, 0) == pdFALSE)
                continue;
            if (xQueuePeek(arg->vel_queue, &tmp_vel, 0) == pdFALSE)
                continue;
    
            if (!is_nil_time(prev_accel.time)) {
                // midpoint Riemann sum
                float prev_fwd = mpu6050_get_fwd_from_total(&(prev_accel.accel));
                float curr_fwd = mpu6050_get_fwd_from_total(&(tmp_accel.accel));
                umicros_t dt_us = absolute_time_diff_us(
                    prev_accel.time, tmp_accel.time
                );

                tmp_vel.vel += ((prev_fwd + curr_fwd) / 2 * MICROS_TO_S(dt_us));
                tmp_vel.time = get_absolute_time();

                xQueueOverwrite(arg->vel_queue, &tmp_vel);
            }

            prev_accel = tmp_accel;
        }
    }
}
