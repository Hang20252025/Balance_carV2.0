#include "app_st_balance_pid.h"

#include "pid_controller.h"

#include <float.h>

#define APP_ST_BALANCE_PID_KP                     35.0f
#define APP_ST_BALANCE_PID_KI                     0.0f
#define APP_ST_BALANCE_PID_KD                     0.9f
#define APP_ST_BALANCE_PID_MAX_OUTPUT             500.0f
#define APP_ST_BALANCE_PID_MAX_INTEGRAL           10.0f
#define APP_ST_BALANCE_PID_DERIVATIVE_FILTER      1.0f
#define APP_ST_BALANCE_PID_MAX_TILT_DEG            30.0f
#define APP_ST_BALANCE_PID_MAX_INTERVAL_MS         30U
#define APP_ST_BALANCE_CONTROL_REVERSE             1

static PID_Config s_balance_pid_config;
static PID_History s_balance_pid_history;
static uint32_t s_last_update_tick;
static uint8_t s_balance_pid_initialized;
static uint8_t s_tilt_fault_latched;

static uint8_t APP_ST_BALANCE_PID_IsFinite(float value)
{
    return (value == value) &&
           (value <= FLT_MAX) &&
           (value >= -FLT_MAX);
}

HAL_StatusTypeDef APP_ST_BALANCE_PID_Init(void)
{
    const PID_Config config =
    {
        APP_ST_BALANCE_PID_KP,
        APP_ST_BALANCE_PID_KI,
        APP_ST_BALANCE_PID_KD,
        -APP_ST_BALANCE_PID_MAX_OUTPUT,
        APP_ST_BALANCE_PID_MAX_OUTPUT,
        -APP_ST_BALANCE_PID_MAX_INTEGRAL,
        APP_ST_BALANCE_PID_MAX_INTEGRAL,
        APP_ST_BALANCE_PID_DERIVATIVE_FILTER
    };

    if (PID_ConfigValidate(&config) != PID_STATUS_OK)
    {
        s_balance_pid_initialized = 0U;
        return HAL_ERROR;
    }

    s_balance_pid_config = config;
    if (PID_HistoryReset(&s_balance_pid_history) != PID_STATUS_OK)
    {
        s_balance_pid_initialized = 0U;
        return HAL_ERROR;
    }

    s_last_update_tick = HAL_GetTick();
    s_balance_pid_initialized = 1U;
    s_tilt_fault_latched = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_BALANCE_PID_Stop(void)
{
    if (s_balance_pid_initialized == 0U)
    {
        return HAL_ERROR;
    }

    if (PID_HistoryReset(&s_balance_pid_history) != PID_STATUS_OK)
    {
        return HAL_ERROR;
    }

    s_last_update_tick = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_BALANCE_PID_Update(
    const APP_ST_IMU_Data *imu_data,
    float target_pitch_deg,
    float *drive_output)
{
    uint32_t now_tick;
    uint32_t elapsed_ms;
    float output;
    PID_Status pid_status;

    if ((s_balance_pid_initialized == 0U) ||
        (imu_data == NULL) ||
        (drive_output == NULL))
    {
        return HAL_ERROR;
    }

    *drive_output = 0.0f;
    if (s_tilt_fault_latched != 0U)
    {
        return HAL_BUSY;
    }

    now_tick = HAL_GetTick();
    elapsed_ms = now_tick - s_last_update_tick;
    s_last_update_tick = now_tick;

    if ((elapsed_ms == 0U) ||
        (elapsed_ms > APP_ST_BALANCE_PID_MAX_INTERVAL_MS))
    {
        return (APP_ST_BALANCE_PID_Stop() == HAL_OK) ? HAL_BUSY : HAL_ERROR;
    }

    if (!APP_ST_BALANCE_PID_IsFinite(imu_data->pitch_deg) ||
        !APP_ST_BALANCE_PID_IsFinite(imu_data->gyro_y_deg_s) ||
        !APP_ST_BALANCE_PID_IsFinite(target_pitch_deg) ||
        (imu_data->pitch_deg > APP_ST_BALANCE_PID_MAX_TILT_DEG) ||
        (imu_data->pitch_deg < -APP_ST_BALANCE_PID_MAX_TILT_DEG))
    {
        if (APP_ST_BALANCE_PID_IsFinite(imu_data->pitch_deg) &&
            ((imu_data->pitch_deg > APP_ST_BALANCE_PID_MAX_TILT_DEG) ||
             (imu_data->pitch_deg < -APP_ST_BALANCE_PID_MAX_TILT_DEG)))
        {
            s_tilt_fault_latched = 1U;
        }

        return (APP_ST_BALANCE_PID_Stop() == HAL_OK) ? HAL_BUSY : HAL_ERROR;
    }

    pid_status = PID_CalculateWithDerivative(
        &s_balance_pid_config,
        imu_data->pitch_deg,
        target_pitch_deg,
        -imu_data->gyro_y_deg_s,
        (float)elapsed_ms * 0.001f,
        &s_balance_pid_history,
        &output);
    if (pid_status != PID_STATUS_OK)
    {
        (void)APP_ST_BALANCE_PID_Stop();
        return HAL_ERROR;
    }

#if APP_ST_BALANCE_CONTROL_REVERSE
    output = -output;
#endif
    *drive_output = output;
    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_BALANCE_PID_SetKp(float kp)
{
    if ((s_balance_pid_initialized == 0U) ||
        !APP_ST_BALANCE_PID_IsFinite(kp))
    {
        return HAL_ERROR;
    }

    s_balance_pid_config.kp = kp;
    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_BALANCE_PID_SetKi(float ki)
{
    if ((s_balance_pid_initialized == 0U) ||
        !APP_ST_BALANCE_PID_IsFinite(ki))
    {
        return HAL_ERROR;
    }

    s_balance_pid_config.ki = ki;
    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_BALANCE_PID_SetKd(float kd)
{
    if ((s_balance_pid_initialized == 0U) ||
        !APP_ST_BALANCE_PID_IsFinite(kd))
    {
        return HAL_ERROR;
    }

    s_balance_pid_config.kd = kd;
    return HAL_OK;
}
