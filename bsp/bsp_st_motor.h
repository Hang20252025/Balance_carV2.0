#ifndef BSP_ST_MOTOR_H
#define BSP_ST_MOTOR_H

#include <stddef.h>
#include <stdint.h>
#include "tim.h"

/* SetPWM accepts values from 0 to this maximum, independent of timer ARR. */
#define BSP_ST_MOTOR_PWM_MAX 1000U

/* Match each timer handle/channel to its PWM pin configured in CubeMX. */
#define BSP_ST_MOTOR_1_TIM_HANDLE  (&htim2)
#define BSP_ST_MOTOR_1_TIM_CHANNEL TIM_CHANNEL_1
#define BSP_ST_MOTOR_1_DIR1_GPIO_PORT GPIOB
#define BSP_ST_MOTOR_1_DIR1_GPIO_PIN  GPIO_PIN_12
#define BSP_ST_MOTOR_1_DIR2_GPIO_PORT GPIOB
#define BSP_ST_MOTOR_1_DIR2_GPIO_PIN  GPIO_PIN_13


#define BSP_ST_MOTOR_2_TIM_HANDLE  (&htim2)
#define BSP_ST_MOTOR_2_TIM_CHANNEL TIM_CHANNEL_2
#define BSP_ST_MOTOR_2_DIR1_GPIO_PORT GPIOB
#define BSP_ST_MOTOR_2_DIR1_GPIO_PIN  GPIO_PIN_14
#define BSP_ST_MOTOR_2_DIR2_GPIO_PORT GPIOB
#define BSP_ST_MOTOR_2_DIR2_GPIO_PIN  GPIO_PIN_15


/* Set both macros when assigning these motors to configured PWM channels. */
#define BSP_ST_MOTOR_3_TIM_HANDLE  NULL
#define BSP_ST_MOTOR_3_TIM_CHANNEL 0U
#define BSP_ST_MOTOR_3_DIR1_GPIO_PORT NULL
#define BSP_ST_MOTOR_3_DIR1_GPIO_PIN  0U
#define BSP_ST_MOTOR_3_DIR2_GPIO_PORT NULL
#define BSP_ST_MOTOR_3_DIR2_GPIO_PIN  0U


#define BSP_ST_MOTOR_4_TIM_HANDLE  NULL
#define BSP_ST_MOTOR_4_TIM_CHANNEL 0U
#define BSP_ST_MOTOR_4_DIR1_GPIO_PORT NULL
#define BSP_ST_MOTOR_4_DIR1_GPIO_PIN  0U
#define BSP_ST_MOTOR_4_DIR2_GPIO_PORT NULL
#define BSP_ST_MOTOR_4_DIR2_GPIO_PIN  0U

//把接口参数都写出来
typedef enum
{
    BSP_ST_MOTOR_ID_1 = 1,
    BSP_ST_MOTOR_ID_2,
    BSP_ST_MOTOR_ID_3,
    BSP_ST_MOTOR_ID_4
} BSP_ST_MotorId;

HAL_StatusTypeDef BSP_ST_Motor_Init(void);
HAL_StatusTypeDef BSP_ST_Motor_SetPWM(BSP_ST_MotorId motor_id, uint16_t pwm_value);
HAL_StatusTypeDef BSP_ST_Motor_SetDirection(
    BSP_ST_MotorId motor_id,
    GPIO_PinState dir1_level,
    GPIO_PinState dir2_level);

#endif /* BSP_ST_MOTOR_H */