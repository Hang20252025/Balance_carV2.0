# `app_st_drive` 使用说明

`app_st_drive` 将平衡环的基础输出和转向环的差速输出组合成左右电机命令，并通过 [`bsp_st_motor.h`](../bsp/bsp_st_motor.h) 输出方向与 PWM。接口位于 [`app_st_drive.h`](./app_st_drive.h)，实现位于 [`app_st_drive.c`](./app_st_drive.c)。

## 初始化与调用

在 GPIO、定时器及其他外设完成初始化后调用：

```c
#include "app_st_drive.h"

if (APP_ST_DRIVE_Init() != HAL_OK)
{
    Error_Handler();
}
```

每次控制周期将两个环的输出传入混控接口：

```c
if (APP_ST_DRIVE_Apply(balance_output, turn_output) != HAL_OK)
{
    Error_Handler();
}
```

混控关系为：

```text
电机 1 = balance_output + turn_output
电机 2 = balance_output - turn_output
```

两个结果分别限制在 `-BSP_ST_MOTOR_PWM_MAX` 到 `+BSP_ST_MOTOR_PWM_MAX`。正负号决定电机方向，绝对值决定 PWM；方向改变时模块先将该电机 PWM 置零，再切换方向。默认两个电机使用相同正向定义；若实际安装相反，在 `app_st_drive.c` 中分别调整 `APP_ST_DRIVE_MOTOR_1_REVERSE` 和 `APP_ST_DRIVE_MOTOR_2_REVERSE`。

调用 `APP_ST_DRIVE_Stop()` 可将两路 PWM 置零。模块未初始化、输入不是有效有限数值或底层电机操作失败时返回 `HAL_ERROR`；运行中发生输入或电机操作错误时会尝试停止两路电机。

## 安全注意

输出限幅仅保护 PWM 指令范围，不会自动验证电机方向是否有助于平衡。首次运行时先架空车轮，检查平衡与转向输出方向；若车体倾倒趋势被加剧，应立即断电并检查 IMU 轴向、控制输出极性和左右电机反向配置。
