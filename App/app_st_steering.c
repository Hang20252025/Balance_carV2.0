#include "app_st_steering.h"

static PID_Config s_steering_pid_config;
static PID_History s_steering_pid_history;
static uint8_t s_steering_pid_initialized;

HAL_StatusTypeDef APP_ST_STEERING_Init(const PID_Config *config)
{
    if (PID_ConfigValidate(config) != PID_STATUS_OK)
    {
        s_steering_pid_initialized = 0U;
        return HAL_ERROR;
    }

    s_steering_pid_config = *config;
    if (PID_HistoryReset(&s_steering_pid_history) != PID_STATUS_OK)
    {
        s_steering_pid_initialized = 0U;
        return HAL_ERROR;
    }

    s_steering_pid_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_STEERING_Reset(void)
{
    if (s_steering_pid_initialized == 0U)
    {
        return HAL_ERROR;
    }

    if (PID_HistoryReset(&s_steering_pid_history) != PID_STATUS_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_STEERING_Update(
    float target_yaw_deg,
    float measured_yaw_deg,
    float dt_seconds,
    float *turn_output)
{
    float error;
    float adjusted_target_yaw_deg;

    if (turn_output != NULL)
    {
        *turn_output = 0.0f;
    }

    if ((s_steering_pid_initialized == 0U) ||
        (turn_output == NULL) ||
        (target_yaw_deg < -180.0f) ||
        (target_yaw_deg > 180.0f) ||
        (measured_yaw_deg < -180.0f) ||
        (measured_yaw_deg > 180.0f))
    {
        return HAL_ERROR;
    }

    error = target_yaw_deg - measured_yaw_deg;
    if (error > 180.0f)
    {
        error -= 360.0f;
    }
    else if (error < -180.0f)
    {
        error += 360.0f;
    }

    adjusted_target_yaw_deg = measured_yaw_deg + error;
    if (PID_Calculate(
            &s_steering_pid_config,
            measured_yaw_deg,
            adjusted_target_yaw_deg,
            dt_seconds,
            &s_steering_pid_history,
            turn_output) != PID_STATUS_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
