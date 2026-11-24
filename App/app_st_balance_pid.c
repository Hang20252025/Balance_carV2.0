#include "app_st_balance_pid.h"

#include "bsp_st_motor.h"

#include <float.h>

#define APP_ST_BALANCE_PID_TARGET_ANGLE_DEG       0.0f
#define APP_ST_BALANCE_PID_KP                     35.0f
#define APP_ST_BALANCE_PID_KI                     0.0f
#define APP_ST_BALANCE_PID_KD                     0.9f
#define APP_ST_BALANCE_PID_MAX_OUTPUT             500.0f
#define APP_ST_BALANCE_PID_MAX_TILT_DEG           30.0f
#define APP_ST_BALANCE_PID_MAX_INTEGRAL            10.0f
#define APP_ST_BALANCE_PID_MAX_INTERVAL_MS         30U
#define APP_ST_BALANCE_CONTROL_REVERSE             0
#define APP_ST_BALANCE_MOTOR_1_REVERSE             0
#define APP_ST_BALANCE_MOTOR_2_REVERSE             0

typedef struct
{
    float integral;
    uint32_t last_update_tick;
    uint8_t initialized;
    uint8_t motor_direction_known[2];
    int8_t motor_direction_positive[2];
} APP_ST_BALANCE_PID_State;

static APP_ST_BALANCE_PID_State s_balance_pid;

static float APP_ST_BALANCE_PID_Clamp(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static uint8_t APP_ST_BALANCE_PID_IsFinite(float value)
{
    return (value == value) &&
           (value <= FLT_MAX) &&
           (value >= -FLT_MAX);
}

static HAL_StatusTypeDef APP_ST_BALANCE_PID_SetMotorOutput(
    BSP_ST_MotorId motor_id,
    uint8_t motor_index,
    int32_t output,
    uint8_t reverse)
{
    GPIO_PinState dir1;
    GPIO_PinState dir2;
    uint8_t direction_positive;
    uint16_t pwm;

    if (output == 0)
    {
        s_balance_pid.motor_direction_known[motor_index] = 0U;
        return BSP_ST_Motor_SetPWM(motor_id, 0U);
    }

    direction_positive = (output > 0) ? 1U : 0U;
    if (reverse != 0U)
    {
        direction_positive = (uint8_t)!direction_positive;
    }

    if ((s_balance_pid.motor_direction_known[motor_index] == 0U) ||
        (s_balance_pid.motor_direction_positive[motor_index] !=
         (int8_t)direction_positive))
    {
        if (BSP_ST_Motor_SetPWM(motor_id, 0U) != HAL_OK)
        {
            return HAL_ERROR;
        }

        dir1 = (direction_positive != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        dir2 = (direction_positive != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        if (BSP_ST_Motor_SetDirection(motor_id, dir1, dir2) != HAL_OK)
        {
            return HAL_ERROR;
        }

        s_balance_pid.motor_direction_positive[motor_index] =
            (int8_t)direction_positive;
        s_balance_pid.motor_direction_known[motor_index] = 1U;
    }

    pwm = (uint16_t)((output > 0) ? output : -output);
    return BSP_ST_Motor_SetPWM(motor_id, pwm);
}

HAL_StatusTypeDef APP_ST_BALANCE_PID_Stop(void)
{
    HAL_StatusTypeDef motor_1_status;
    HAL_StatusTypeDef motor_2_status;

    s_balance_pid.integral = 0.0f;
    s_balance_pid.last_update_tick = HAL_GetTick();
    s_balance_pid.motor_direction_known[0] = 0U;
    s_balance_pid.motor_direction_known[1] = 0U;

    if (s_balance_pid.initialized == 0U)
    {
        return HAL_ERROR;
    }

    motor_1_status = BSP_ST_Motor_SetPWM(BSP_ST_MOTOR_ID_1, 0U);
    motor_2_status = BSP_ST_Motor_SetPWM(BSP_ST_MOTOR_ID_2, 0U);

    return ((motor_1_status == HAL_OK) && (motor_2_status == HAL_OK))
               ? HAL_OK
               : HAL_ERROR;
}

HAL_StatusTypeDef APP_ST_BALANCE_PID_Init(void)
{
    HAL_StatusTypeDef status;

    s_balance_pid.integral = 0.0f;
    s_balance_pid.last_update_tick = HAL_GetTick();
    s_balance_pid.initialized = 0U;
    s_balance_pid.motor_direction_known[0] = 0U;
    s_balance_pid.motor_direction_known[1] = 0U;

    status = BSP_ST_Motor_Init();
    if (status != HAL_OK)
    {
        return status;
    }

    s_balance_pid.initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_BALANCE_PID_Update(const APP_ST_IMU_Data *imu_data)
{
    uint32_t now_tick;
    uint32_t elapsed_ms;
    float dt_seconds;
    float error;
    float output;
    int32_t motor_output;
    HAL_StatusTypeDef status;

    if ((s_balance_pid.initialized == 0U) || (imu_data == NULL))
    {
        return HAL_ERROR;
    }

    now_tick = HAL_GetTick();
    elapsed_ms = now_tick - s_balance_pid.last_update_tick;
    s_balance_pid.last_update_tick = now_tick;

    if ((elapsed_ms == 0U) ||
        (elapsed_ms > APP_ST_BALANCE_PID_MAX_INTERVAL_MS))
    {
        status = APP_ST_BALANCE_PID_Stop();
        return (status == HAL_OK) ? HAL_BUSY : HAL_ERROR;
    }

    if (!APP_ST_BALANCE_PID_IsFinite(imu_data->pitch_deg) ||
        !APP_ST_BALANCE_PID_IsFinite(imu_data->gyro_y_deg_s) ||
        (imu_data->pitch_deg > APP_ST_BALANCE_PID_MAX_TILT_DEG) ||
        (imu_data->pitch_deg < -APP_ST_BALANCE_PID_MAX_TILT_DEG))
    {
        status = APP_ST_BALANCE_PID_Stop();
        return (status == HAL_OK) ? HAL_BUSY : HAL_ERROR;
    }

    dt_seconds = (float)elapsed_ms * 0.001f;
    error = imu_data->pitch_deg - APP_ST_BALANCE_PID_TARGET_ANGLE_DEG;
    s_balance_pid.integral = APP_ST_BALANCE_PID_Clamp(
        s_balance_pid.integral + (error * dt_seconds),
        -APP_ST_BALANCE_PID_MAX_INTEGRAL,
        APP_ST_BALANCE_PID_MAX_INTEGRAL);

    output = (APP_ST_BALANCE_PID_KP * error) +
             (APP_ST_BALANCE_PID_KI * s_balance_pid.integral) +
             (APP_ST_BALANCE_PID_KD * imu_data->gyro_y_deg_s);
#if APP_ST_BALANCE_CONTROL_REVERSE
    output = -output;
#endif
    output = APP_ST_BALANCE_PID_Clamp(
        output,
        -APP_ST_BALANCE_PID_MAX_OUTPUT,
        APP_ST_BALANCE_PID_MAX_OUTPUT);
    motor_output = (int32_t)((output >= 0.0f) ? (output + 0.5f)
                                               : (output - 0.5f));

    status = APP_ST_BALANCE_PID_SetMotorOutput(
        BSP_ST_MOTOR_ID_1,
        0U,
        motor_output,
        APP_ST_BALANCE_MOTOR_1_REVERSE);
    if (status != HAL_OK)
    {
        (void)APP_ST_BALANCE_PID_Stop();
        return status;
    }

    status = APP_ST_BALANCE_PID_SetMotorOutput(
        BSP_ST_MOTOR_ID_2,
        1U,
        motor_output,
        APP_ST_BALANCE_MOTOR_2_REVERSE);
    if (status != HAL_OK)
    {
        (void)APP_ST_BALANCE_PID_Stop();
    }

    return status;
}
