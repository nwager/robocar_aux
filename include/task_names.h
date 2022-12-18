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

static const char PROCESS_MASTER_TASK_NAME[] = "Process_Master_Task";
static const char MPU6050_TASK_NAME[] = "MPU6050_Task";
static const char UPDATE_VEL_TASK_NAME[] = "Update_Vel_Task";

#endif /* TASK_NAMES_H */