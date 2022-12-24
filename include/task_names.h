/**
 * @file task_names.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief Defines constant string names of each FreeRTOS task so they
 *     work with intellisense and autocomplete.
 * @version 0.1
 * @date 2022-12-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef TASK_NAMES_H
#define TASK_NAMES_H

static const char PROCESS_BRAIN_TASK_NAME[] = "Process_Brain_Task";
static const char MPU6050_TASK_NAME[] = "MPU6050_Task";
static const char UPDATE_VEL_TASK_NAME[] = "Update_Vel_Task";
static const char PID_VEL_TASK_NAME[] = "PID_Vel_Task";
static const char STEERING_TASK_NAME[] = "Steering_Task";
static const char ACTUATION_TASK_NAME[] = "Actuation_Task";

#endif /* TASK_NAMES_H */