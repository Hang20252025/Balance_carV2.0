# `app_st_steering` 使用说明

`app_st_steering` 是航向/转向控制环：根据目标航向和测量航向调用通用 PID，输出供左右轮差速混控使用的转向量。接口位于 [`app_st_steering.h`](./app_st_steering.h)，实现位于 [`app_st_steering.c`](./app_st_steering.c)。

该模块不读取陀螺仪或磁力计。调用方需要先计算航向角并提供输入。角度使用度，目标值和测量值必须在 `[-180°, 180°]` 范围内；模块会选择最短的角度误差方向。

## 初始化与更新

```c
#include "app_st_steering.h"

static const PID_Config steering_config =
{
    1.0f, 0.0f, 0.0f,      /* kp, ki, kd：示例值，需实际整定 */
    -250.0f, 250.0f,       /* 输出限幅：转向差速量 */
    -20.0f, 20.0f,         /* 积分限幅 */
    0.5f                   /* 微分滤波系数 */
};
float turn_output;

if (APP_ST_STEERING_Init(&steering_config) != HAL_OK)
{
    Error_Handler();
}

if (APP_ST_STEERING_Update(
        target_yaw_deg,
        measured_yaw_deg,
        dt_seconds,
        &turn_output) != HAL_OK)
{
    Error_Handler();
}
```

成功时，`turn_output` 可传给 `APP_ST_DRIVE_Apply(balance_output, turn_output)`。正负方向取决于航向定义和电机安装方向，需通过架空车轮测试确认。初始化失败、角度超出范围、无效采样周期或未初始化时返回 `HAL_ERROR`，并将非空输出指针置零。

`APP_ST_STEERING_Reset()` 清除该环内部状态。当前模块尚未接入主循环，因为项目还没有可用的航向估计输入。
