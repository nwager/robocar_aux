/**
 * @file brain_interface.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief Handles brain-router serial communication.
 * @version 0.1
 * @date 2022-12-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef BRAIN_INTERFACE_H
#define BRAIN_INTERFACE_H

#include <stdint.h>

#include <queue.h>
#include <semphr.h>

typedef struct {
    QueueHandle_t control_queue;
    QueueHandle_t accel_queue;
    QueueHandle_t vel_queue;
    QueueHandle_t act_queue;
    SemaphoreHandle_t serial_mutex;
} process_brain_task_arg_t;

/**
 * @brief Main communication task with brain.
 * 
 * @param p Pointer to process_brain_task_arg_t.
 */
void process_brain_task(void *p);

#endif /* BRAIN_INTERFACE_H */