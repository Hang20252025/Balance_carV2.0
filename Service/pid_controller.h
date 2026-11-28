#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

typedef enum
{
    PID_STATUS_OK = 0,
    PID_STATUS_INVALID_ARGUMENT
} PID_Status;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float derivative_filter_alpha;
} PID_Config;

typedef struct
{
    float integral;
    float previous_error;
    float filtered_derivative;
    uint8_t has_previous_error;
} PID_History;

PID_Status PID_ConfigValidate(const PID_Config *config);
PID_Status PID_HistoryReset(PID_History *history);

PID_Status PID_Calculate(
    const PID_Config *config,
    float current_value,
    float target_value,
    float dt_seconds,
    PID_History *history,
    float *output);

/* error_derivative is the derivative of (target_value - current_value). */
PID_Status PID_CalculateWithDerivative(
    const PID_Config *config,
    float current_value,
    float target_value,
    float error_derivative,
    float dt_seconds,
    PID_History *history,
    float *output);

#endif /* PID_CONTROLLER_H */
