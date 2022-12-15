/**
 * @file core_comm_queue.h
 * @author Noah Wager (noahwager@gmail.com)
 * @brief Constants and types for intercore communication queues.
 * @version 0.1
 * @date 2022-12-14
 * 
 * @copyright Copyright (c) 2022
 * 
 * Defines types and command values associated with the command and
 * response queues on which core 0 and 1 communicate.
 * 
 */

#ifndef _CORE_COMM_QUEUE_H_
#define _CORE_COMM_QUEUE_H_

/// @brief Command queue message code type.
typedef enum {
    NOOP,
    CALIBRATE_ACCEL,
    GET_SPEED,
    GET_ACCEL,
    RESET_MPU6050,
    CORE_CMD_END,
} core_command_t;

/// @brief Response status code type.
typedef enum {
    SUCCESS,
    FAIL,
} core_response_code_t;

/// @brief Response queue data type.
typedef struct {
    void *payload;
    uint payload_size; // Number of bytes that payload points to.
    core_response_code_t code;
} core_response_t;

#endif // _CORE_COMM_QUEUE_H