# `bsp_st_motor` 使用说明

`bsp_st_motor` 为最多四台电机提供统一的 PWM 和方向控制接口。电机编号、PWM 定时器/通道和方向 GPIO 的映射定义在 [`bsp_st_motor.h`](./bsp_st_motor.h)；实现位于 [`bsp_st_motor.c`](./bsp_st_motor.c)。

## 当前引脚映射

| 电机编号              | PWM 定时器通道 | PWM 引脚 | 方向脚 1 | 方向脚 2 |
| --------------------- | -------------- | -------- | -------- | -------- |
| `BSP_ST_MOTOR_ID_1` | TIM2_CH1       | PA0      | PB12     | PB13     |
| `BSP_ST_MOTOR_ID_2` | TIM2_CH2       | PA1      | PB14     | PB15     |
| `BSP_ST_MOTOR_ID_3` | 未配置         | —       | 未配置   | 未配置   |
| `BSP_ST_MOTOR_ID_4` | 未配置         | —       | 未配置   | 未配置   |

电机 3、4 的接口已保留，但其配置宏目前为 `NULL`/`0U`。配置完成前，对它们调用控制接口会返回 `HAL_ERROR`。

## 初始化

先初始化 CubeMX 生成的 GPIO 和定时器，再初始化电机 BSP：

```c
#include "gpio.h"
#include "bsp_st_motor.h"

/* 在 main() 中，CubeMX 外设初始化之后调用 */
MX_GPIO_Init();
MX_TIM2_Init();

if (BSP_ST_Motor_Init() != HAL_OK)
{
    Error_Handler();
}
```

`BSP_ST_Motor_Init()` 会检查已配置电机的映射，检查定时器通道和方向引脚是否重复，将方向引脚置为低电平、PWM 占空比设为 0，并启动已配置的 PWM 通道。重复调用时会直接返回 `HAL_OK`。

`MX_GPIO_Init()` 负责启用 GPIOB 时钟；`BSP_ST_Motor_Init()` 会将映射的方向引脚初始化为推挽输出。若更改引脚映射，需确保对应 GPIO 端口时钟已启用。

## 设置 PWM

```c
if (BSP_ST_Motor_SetPWM(BSP_ST_MOTOR_ID_1, 500U) != HAL_OK)
{
    Error_Handler();
}
```

`pwm_value` 范围为 `0` 到 `BSP_ST_MOTOR_PWM_MAX`（默认 `1000`）：

- `0`：0% 占空比
- `500`：50% 占空比
- `1000`：100% 占空比

实现会读取当前定时器的 ARR，并按 `(ARR + 1) × pwm_value / BSP_ST_MOTOR_PWM_MAX` 计算比较值，因此调用方不需要根据 ARR 换算输入值。PWM 频率仍由定时器时钟、预分频器和 ARR 决定。

## 设置方向引脚电平

```c
if (BSP_ST_Motor_SetDirection(
        BSP_ST_MOTOR_ID_1,
        GPIO_PIN_SET,
        GPIO_PIN_RESET) != HAL_OK)
{
    Error_Handler();
}
```

两个电平参数分别写入该电机的方向脚 1 和方向脚 2，只接受 `GPIO_PIN_RESET` 或 `GPIO_PIN_SET`。接口不将电平解释为“正转”或“反转”；实际方向以及两脚同电平时的行为取决于电机驱动器接线和驱动器规格。

建议在初始化后按需要设置方向，再设置 PWM。设置方向不会自动改变 PWM 占空比；设置 PWM 也不会改变方向。

## 修改或增加电机映射

在 [`bsp_st_motor.h`](./bsp_st_motor.h) 中为对应电机设置以下宏：

- `BSP_ST_MOTOR_n_TIM_HANDLE` 和 `BSP_ST_MOTOR_n_TIM_CHANNEL`
- `BSP_ST_MOTOR_n_DIR1_GPIO_PORT` 和 `BSP_ST_MOTOR_n_DIR1_GPIO_PIN`
- `BSP_ST_MOTOR_n_DIR2_GPIO_PORT` 和 `BSP_ST_MOTOR_n_DIR2_GPIO_PIN`

例如，方向引脚映射使用 `GPIOB` 与 `GPIO_PIN_x`。更换 PWM 引脚时，应在 CubeMX 中选择与该定时器通道对应的复用功能引脚；仅修改 C 宏不会重新配置 MCU 引脚。确保每个电机使用独立的 PWM 通道和方向 GPIO，并在 CubeMX 中将方向 GPIO 配置为输出。

接口返回 `HAL_OK` 表示操作成功；未初始化、编号无效、参数超范围或电机尚未配置时会返回 `HAL_ERROR`。
