#include "bsp_st_motor.h"

typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    GPIO_TypeDef *dir1_port;
    uint16_t dir1_pin;
    GPIO_TypeDef *dir2_port;
    uint16_t dir2_pin;
} BSP_ST_MotorConfig;

//定义了包含四个电机的配置数组
static const BSP_ST_MotorConfig s_motor_config[] =
{
    {
        BSP_ST_MOTOR_1_TIM_HANDLE,
        BSP_ST_MOTOR_1_TIM_CHANNEL,
        BSP_ST_MOTOR_1_DIR1_GPIO_PORT,
        BSP_ST_MOTOR_1_DIR1_GPIO_PIN,
        BSP_ST_MOTOR_1_DIR2_GPIO_PORT,
        BSP_ST_MOTOR_1_DIR2_GPIO_PIN
    },
    {
        BSP_ST_MOTOR_2_TIM_HANDLE,
        BSP_ST_MOTOR_2_TIM_CHANNEL,
        BSP_ST_MOTOR_2_DIR1_GPIO_PORT,
        BSP_ST_MOTOR_2_DIR1_GPIO_PIN,
        BSP_ST_MOTOR_2_DIR2_GPIO_PORT,
        BSP_ST_MOTOR_2_DIR2_GPIO_PIN
    },
    {
        BSP_ST_MOTOR_3_TIM_HANDLE,
        BSP_ST_MOTOR_3_TIM_CHANNEL,
        BSP_ST_MOTOR_3_DIR1_GPIO_PORT,
        BSP_ST_MOTOR_3_DIR1_GPIO_PIN,
        BSP_ST_MOTOR_3_DIR2_GPIO_PORT,
        BSP_ST_MOTOR_3_DIR2_GPIO_PIN
    },
    {
        BSP_ST_MOTOR_4_TIM_HANDLE,
        BSP_ST_MOTOR_4_TIM_CHANNEL,
        BSP_ST_MOTOR_4_DIR1_GPIO_PORT,
        BSP_ST_MOTOR_4_DIR1_GPIO_PIN,
        BSP_ST_MOTOR_4_DIR2_GPIO_PORT,
        BSP_ST_MOTOR_4_DIR2_GPIO_PIN
    }
};

static uint8_t s_motor_pwm_initialized;

//检查电机配置是否未配置
static uint8_t BSP_ST_Motor_IsUnconfigured(const BSP_ST_MotorConfig *config)
{
    return (config->timer == NULL) &&
           (config->channel == 0U) &&
           (config->dir1_port == NULL) &&
           (config->dir1_pin == 0U) &&
           (config->dir2_port == NULL) &&
           (config->dir2_pin == 0U);
}

static uint8_t BSP_ST_Motor_IsValidChannel(uint32_t channel)
{
    return (channel == TIM_CHANNEL_1) ||
           (channel == TIM_CHANNEL_2) ||
           (channel == TIM_CHANNEL_3) ||
           (channel == TIM_CHANNEL_4);
}

static uint8_t BSP_ST_Motor_IsValidGpioPin(uint16_t pin)
{
    return (pin != 0U) && ((pin & (uint16_t)(pin - 1U)) == 0U);
}

static void BSP_ST_Motor_InitDirectionPin(
    GPIO_TypeDef *port,
    uint16_t pin)
{
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gpio_init);
}

static const BSP_ST_MotorConfig *BSP_ST_Motor_GetConfig(BSP_ST_MotorId motor_id)
{   
    // 防止motor_id非法
    if ((motor_id < BSP_ST_MOTOR_ID_1) || (motor_id > BSP_ST_MOTOR_ID_4))
    {
        return NULL;
    }
    // 返回地址
    return &s_motor_config[(uint32_t)motor_id - 1U];
}

HAL_StatusTypeDef BSP_ST_Motor_Init(void)
{
    uint32_t i;
    uint32_t j;
    uint8_t configured_motor_count = 0U;
// 检查是否已经初始化
    if (s_motor_pwm_initialized != 0U)
    {
        return HAL_OK;
    }

    for (i = 0U; i < (sizeof(s_motor_config) / sizeof(s_motor_config[0])); i++)
    {
        const BSP_ST_MotorConfig *config = &s_motor_config[i];

        if (BSP_ST_Motor_IsUnconfigured(config))
        {
            continue;
        }

        if ((config->timer == NULL) ||
            (config->timer->Instance == NULL) ||
            !BSP_ST_Motor_IsValidChannel(config->channel) ||
            (config->dir1_port == NULL) ||
            !BSP_ST_Motor_IsValidGpioPin(config->dir1_pin) ||
            (config->dir2_port == NULL) ||
            !BSP_ST_Motor_IsValidGpioPin(config->dir2_pin) ||
            ((config->dir1_port == config->dir2_port) &&
             ((config->dir1_pin & config->dir2_pin) != 0U)))
        {
            return HAL_ERROR;
        }

        configured_motor_count++;

        for (j = i + 1U; j < (sizeof(s_motor_config) / sizeof(s_motor_config[0])); j++)
        {
            const BSP_ST_MotorConfig *other = &s_motor_config[j];

            if ((other->timer == config->timer) &&
                (other->channel == config->channel))
            {
                return HAL_ERROR;
            }

            if (((other->dir1_port == config->dir1_port) &&
                 ((other->dir1_pin & config->dir1_pin) != 0U)) ||
                ((other->dir1_port == config->dir2_port) &&
                 ((other->dir1_pin & config->dir2_pin) != 0U)) ||
                ((other->dir2_port == config->dir1_port) &&
                 ((other->dir2_pin & config->dir1_pin) != 0U)) ||
                ((other->dir2_port == config->dir2_port) &&
                 ((other->dir2_pin & config->dir2_pin) != 0U)))
            {
                return HAL_ERROR;
            }
        }
    }

    if (configured_motor_count == 0U)
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < (sizeof(s_motor_config) / sizeof(s_motor_config[0])); i++)
    {
        const BSP_ST_MotorConfig *config = &s_motor_config[i];
        HAL_StatusTypeDef status;

        if (config->timer == NULL)
        {
            continue;
        }

        BSP_ST_Motor_InitDirectionPin(config->dir1_port, config->dir1_pin);
        BSP_ST_Motor_InitDirectionPin(config->dir2_port, config->dir2_pin);
        HAL_GPIO_WritePin(config->dir1_port, config->dir1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(config->dir2_port, config->dir2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(config->timer, config->channel, 0U);
        status = HAL_TIM_PWM_Start(config->timer, config->channel);
        if (status != HAL_OK)
        {
            return status;
        }
    }

    s_motor_pwm_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef BSP_ST_Motor_SetPWM(BSP_ST_MotorId motor_id, uint16_t pwm_value)
{
    const BSP_ST_MotorConfig *config = BSP_ST_Motor_GetConfig(motor_id);
    uint32_t auto_reload;
    uint64_t compare_value;
    uint64_t timer_period;

    if ((s_motor_pwm_initialized == 0U) ||
        (config == NULL) ||
        (config->timer == NULL) ||
        (pwm_value > BSP_ST_MOTOR_PWM_MAX))
    {
        return HAL_ERROR;
    }

    auto_reload = __HAL_TIM_GET_AUTORELOAD(config->timer);
    timer_period = (uint64_t)auto_reload + 1U;
    compare_value = (timer_period * pwm_value) / BSP_ST_MOTOR_PWM_MAX;
    if (compare_value > UINT32_MAX)
    {
        compare_value = UINT32_MAX;
    }

    __HAL_TIM_SET_COMPARE(config->timer, config->channel, (uint32_t)compare_value);
    return HAL_OK;
}

HAL_StatusTypeDef BSP_ST_Motor_SetDirection(
    BSP_ST_MotorId motor_id,
    GPIO_PinState dir1_level,
    GPIO_PinState dir2_level)
{
    const BSP_ST_MotorConfig *config = BSP_ST_Motor_GetConfig(motor_id);

    if ((s_motor_pwm_initialized == 0U) ||
        (config == NULL) ||
        (config->timer == NULL) ||
        (config->dir1_port == NULL) ||
        !BSP_ST_Motor_IsValidGpioPin(config->dir1_pin) ||
        (config->dir2_port == NULL) ||
        !BSP_ST_Motor_IsValidGpioPin(config->dir2_pin) ||
        ((dir1_level != GPIO_PIN_RESET) && (dir1_level != GPIO_PIN_SET)) ||
        ((dir2_level != GPIO_PIN_RESET) && (dir2_level != GPIO_PIN_SET)))
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(config->dir1_port, config->dir1_pin, dir1_level);
    HAL_GPIO_WritePin(config->dir2_port, config->dir2_pin, dir2_level);

    return HAL_OK;
}