#include "app_st_drive.h"

#include "bsp_st_motor.h"

#include <float.h>

#define APP_ST_DRIVE_MAX_OUTPUT       ((float)BSP_ST_MOTOR_PWM_MAX)
#define APP_ST_DRIVE_MOTOR_1_REVERSE  0U
#define APP_ST_DRIVE_MOTOR_2_REVERSE  0U

static uint8_t s_drive_initialized;
static uint8_t s_motor_direction_known[2];
static uint8_t s_motor_direction_positive[2];

static uint8_t APP_ST_DRIVE_IsFinite(float value)
{
    return (value == value) &&
           (value <= FLT_MAX) &&
           (value >= -FLT_MAX);
}

static float APP_ST_DRIVE_Clamp(float value)
{
    if (value > APP_ST_DRIVE_MAX_OUTPUT)
    {
        return APP_ST_DRIVE_MAX_OUTPUT;
    }

    if (value < -APP_ST_DRIVE_MAX_OUTPUT)
    {
        return -APP_ST_DRIVE_MAX_OUTPUT;
    }

    return value;
}

static HAL_StatusTypeDef APP_ST_DRIVE_SetMotor(
    BSP_ST_MotorId motor_id,
    uint8_t motor_index,
    float output,
    uint8_t reverse)
{
    uint8_t direction_positive;
    GPIO_PinState dir1;
    GPIO_PinState dir2;
    uint16_t pwm;

    if (output == 0.0f)
    {
        s_motor_direction_known[motor_index] = 0U;
        return BSP_ST_Motor_SetPWM(motor_id, 0U);
    }

    direction_positive = (output > 0.0f) ? 1U : 0U;
    if (reverse != 0U)
    {
        direction_positive = (uint8_t)!direction_positive;
    }

    if ((s_motor_direction_known[motor_index] == 0U) ||
        (s_motor_direction_positive[motor_index] != direction_positive))
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

        s_motor_direction_positive[motor_index] = direction_positive;
        s_motor_direction_known[motor_index] = 1U;
    }

    pwm = (uint16_t)((output >= 0.0f) ? (output + 0.5f)
                                      : (-output + 0.5f));
    return BSP_ST_Motor_SetPWM(motor_id, pwm);
}

HAL_StatusTypeDef APP_ST_DRIVE_Init(void)
{
    HAL_StatusTypeDef motor_1_status;
    HAL_StatusTypeDef motor_2_status;

    s_drive_initialized = 0U;
    s_motor_direction_known[0] = 0U;
    s_motor_direction_known[1] = 0U;

    if (BSP_ST_Motor_Init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_drive_initialized = 1U;
    motor_1_status = BSP_ST_Motor_SetPWM(BSP_ST_MOTOR_ID_1, 0U);
    motor_2_status = BSP_ST_Motor_SetPWM(BSP_ST_MOTOR_ID_2, 0U);

    if ((motor_1_status != HAL_OK) || (motor_2_status != HAL_OK))
    {
        (void)APP_ST_DRIVE_Stop();
        s_drive_initialized = 0U;
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_DRIVE_Stop(void)
{
    HAL_StatusTypeDef motor_1_status;
    HAL_StatusTypeDef motor_2_status;

    if (s_drive_initialized == 0U)
    {
        return HAL_ERROR;
    }

    s_motor_direction_known[0] = 0U;
    s_motor_direction_known[1] = 0U;
    motor_1_status = BSP_ST_Motor_SetPWM(BSP_ST_MOTOR_ID_1, 0U);
    motor_2_status = BSP_ST_Motor_SetPWM(BSP_ST_MOTOR_ID_2, 0U);

    return ((motor_1_status == HAL_OK) && (motor_2_status == HAL_OK))
               ? HAL_OK
               : HAL_ERROR;
}

HAL_StatusTypeDef APP_ST_DRIVE_Apply(float balance_output, float turn_output)
{
    float motor_1_output;
    float motor_2_output;
    HAL_StatusTypeDef status;

    if (s_drive_initialized == 0U)
    {
        return HAL_ERROR;
    }

    if (!APP_ST_DRIVE_IsFinite(balance_output) ||
        !APP_ST_DRIVE_IsFinite(turn_output))
    {
        (void)APP_ST_DRIVE_Stop();
        return HAL_ERROR;
    }

    motor_1_output = APP_ST_DRIVE_Clamp(balance_output + turn_output);
    motor_2_output = APP_ST_DRIVE_Clamp(balance_output - turn_output);

    status = APP_ST_DRIVE_SetMotor(
        BSP_ST_MOTOR_ID_1,
        0U,
        motor_1_output,
        APP_ST_DRIVE_MOTOR_1_REVERSE);
    if (status != HAL_OK)
    {
        (void)APP_ST_DRIVE_Stop();
        return status;
    }

    status = APP_ST_DRIVE_SetMotor(
        BSP_ST_MOTOR_ID_2,
        1U,
        motor_2_output,
        APP_ST_DRIVE_MOTOR_2_REVERSE);
    if (status != HAL_OK)
    {
        (void)APP_ST_DRIVE_Stop();
    }

    return status;
}
