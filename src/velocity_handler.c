#include <stdio.h>
#include <stdlib.h>
#include <limits.h> // for ULONG_MAX

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
// generated pio header
#include "wheel_rev.pio.h"

#include "type_utils.h"
#include "mpu6050.h"
#include "math_utils.h"

#include "velocity_handler.h"

#define WHEEL_REV_GPIO 15

#define WHEEL_REV_PIO pio0
#define WHEEL_REV_PIO_IRQ_NUM 0
#define WHEEL_REV_CPU_IRQ PIO0_IRQ_0

#define MICROS_TO_S(t) (((float)(t)) / 1000000.0)

#define WHEEL_R_M (49.68 / 1000)
#define WHEEL_C_M (2.0 * PI * WHEEL_R_M)

static absolute_time_t prev_time, curr_time;
static TaskHandle_t update_vel_task_handle;

/**
 * @brief Interrupt handler for wheel revolution detection.
 * 
 */
static void wheel_rev_isr();
/**
 * @brief Initializes wheel revolution detector PIO.
 * 
 */
static void init_wheel_rev_pio();

void update_vel_task(void *p) {
    update_vel_task_handle = xTaskGetCurrentTaskHandle();

    curr_time = nil_time;
    prev_time = nil_time;

    velocity_stamped_t tmp_vel;

    accel_stamped_t prev_accel, tmp_accel;
    prev_accel.time = nil_time;

    // pio
    init_wheel_rev_pio();

    update_vel_task_arg_t *arg = p;
    uint32_t notif_val;

    while (1) {
        xTaskNotifyWait(0, ULONG_MAX, &notif_val, portMAX_DELAY);

        if ((notif_val & WHEEL_NOTIF_MASK) && !is_nil_time(prev_time)) {
            
            // process new wheel measurement
            umicros_t dt_us = absolute_time_diff_us(prev_time, curr_time);

            tmp_vel.vel = WHEEL_C_M / MICROS_TO_S(dt_us);
            tmp_vel.time = get_absolute_time();

            xQueueOverwrite(arg->vel_queue, &tmp_vel);
        } else if (notif_val & ACCEL_NOTIF_MASK) {
            // no wheel measurement, process new acceleration measurement
            
            // get current acceleration
            if (xQueuePeek(arg->accel_queue, &tmp_accel, 0) == pdFALSE)
                continue;
            // get current velocity
            if (xQueuePeek(arg->vel_queue, &tmp_vel, 0) == pdFALSE)
                continue;
    
            if (!is_nil_time(prev_accel.time)
                && abs(mpu6050_get_fwd_from_total(&(tmp_accel.accel))) > 0.5) {
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

static void wheel_rev_isr() {
    pio_interrupt_clear(WHEEL_REV_PIO, WHEEL_REV_PIO_IRQ_NUM);
    prev_time = curr_time;
    curr_time = get_absolute_time();
    xTaskNotifyFromISR(
        update_vel_task_handle,
        WHEEL_NOTIF_MASK,
        eSetBits,
        NULL
    );
}

static void init_wheel_rev_pio() {
    PIO pio = pio0;
    uint pio_irq = PIO0_IRQ_0;

    uint sm = pio_claim_unused_sm(WHEEL_REV_PIO, true);
    uint offset = pio_add_program(WHEEL_REV_PIO, &wheel_rev_program);

    // set pio interrupt handler
    irq_set_exclusive_handler(WHEEL_REV_CPU_IRQ, wheel_rev_isr);
    // enable pio interrupt for cpu
    irq_set_enabled(WHEEL_REV_CPU_IRQ, true);
    
    wheel_rev_program_init(WHEEL_REV_PIO, sm, offset, WHEEL_REV_GPIO);
}
