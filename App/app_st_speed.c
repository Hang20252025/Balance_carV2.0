#include "app_st_speed.h"

static PID_Config s_speed_pid_config;
static PID_History s_speed_pid_history;
static uint8_t s_speed_pid_initialized;

HAL_StatusTypeDef APP_ST_SPEED_Init(const PID_Config *config)
{
    if (PID_ConfigValidate(config) != PID_STATUS_OK)
    {
        s_speed_pid_initialized = 0U;
        return HAL_ERROR;
    }

    s_speed_pid_config = *config;
    if (PID_HistoryReset(&s_speed_pid_history) != PID_STATUS_OK)
    {
        s_speed_pid_initialized = 0U;
        return HAL_ERROR;
    }

    s_speed_pid_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_SPEED_Reset(void)
{
    if (s_speed_pid_initialized == 0U)
    {
        return HAL_ERROR;
    }

    if (PID_HistoryReset(&s_speed_pid_history) != PID_STATUS_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_SPEED_Update(
    float target_speed,
    float measured_speed,
    float dt_seconds,
    float *target_pitch_deg)
{
    if (target_pitch_deg != NULL)
    {
        *target_pitch_deg = 0.0f;
    }

    if ((s_speed_pid_initialized == 0U) ||
        (PID_Calculate(
             &s_speed_pid_config,
             measured_speed,
             target_speed,
             dt_seconds,
             &s_speed_pid_history,
             target_pitch_deg) != PID_STATUS_OK))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
