# `app_st_speed` 使用说明

`app_st_speed` 是速度外环：根据目标速度和测量速度调用通用 PID，输出平衡环的目标俯仰角。接口位于 [`app_st_speed.h`](./app_st_speed.h)，实现位于 [`app_st_speed.c`](./app_st_speed.c)。

该模块不读取编码器，也不控制电机。调用方负责提供速度反馈、目标速度和真实采样周期。速度和输出角度的单位由项目统一约定，例如速度用 `m/s`、目标俯仰角用度。

## 初始化与更新

```c
#include "app_st_speed.h"

static const PID_Config speed_config =
{
    1.0f, 0.0f, 0.0f,  /* kp, ki, kd：示例值，需实际整定 */
    -5.0f, 5.0f,        /* 输出限幅：目标俯仰角 */
    -2.0f, 2.0f,        /* 积分限幅 */
    0.5f                /* 微分滤波系数 */
};
float target_pitch_deg;

if (APP_ST_SPEED_Init(&speed_config) != HAL_OK)
{
    Error_Handler();
}

if (APP_ST_SPEED_Update(
        target_speed,
        measured_speed,
        dt_seconds,
        &target_pitch_deg) != HAL_OK)
{
    Error_Handler();
}
```

模块使用 `target_speed - measured_speed` 作为 PID 误差。成功时，`target_pitch_deg` 是送入平衡环的目标角；无效参数、非正 `dt_seconds` 或未初始化时返回 `HAL_ERROR`，并将非空输出指针置零。

`APP_ST_SPEED_Reset()` 清除该环的积分和微分状态，适用于切换工作模式或速度设定后需要重新开始积分的情况。该模块当前未接入主循环，因为工程尚未提供编码器速度反馈。
