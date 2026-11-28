# `pid_controller` 使用说明

`pid_controller` 是与 STM32 HAL 和传感器无关的通用位置式 PID 算法模块。每次计算都显式传入当前测量值、目标值、PID 配置、采样间隔和历史状态；历史状态由调用方持有并在计算成功时更新。接口位于 [`pid_controller.h`](./pid_controller.h)，实现位于 [`pid_controller.c`](./pid_controller.c)。

## 配置与历史值

```c
#include "pid_controller.h"

static const PID_Config speed_config =
{
    1.0f,   /* kp */
    0.0f,   /* ki */
    0.0f,   /* kd */
    -5.0f,  /* output_min */
    5.0f,   /* output_max */
    -2.0f,  /* integral_min */
    2.0f,   /* integral_max */
    0.5f    /* derivative_filter_alpha */
};
static PID_History speed_history;

if ((PID_ConfigValidate(&speed_config) != PID_STATUS_OK) ||
    (PID_HistoryReset(&speed_history) != PID_STATUS_OK))
{
    Error_Handler();
}
```

配置字段：

| 字段                               | 说明                                                        |
| ---------------------------------- | ----------------------------------------------------------- |
| `kp`、`ki`、`kd`             | 比例、积分、微分系数，需与误差和输出量纲匹配                |
| `output_min`、`output_max`     | 输出限幅，必须满足最小值小于最大值                          |
| `integral_min`、`integral_max` | 积分状态限幅，最小值不得大于最大值                          |
| `derivative_filter_alpha`        | 微分低通滤波系数，范围为 0 到 1；1 表示不滤波，越小滤波越强 |

`PID_History` 保存积分、上次误差、滤波后的微分值和历史有效标志。静态或零初始化的历史值可以直接使用；调用 `PID_HistoryReset()` 可清空历史并重新开始计算。每个控制环必须分别保存自己的历史值。

## 计算输出

默认误差为 `target_value - current_value`，微分由相邻误差和采样间隔计算：

```c
float output;
float dt_seconds = 0.005f;

if (PID_Calculate(
        &speed_config,
        measured_speed,
        target_speed,
        dt_seconds,
        &speed_history,
        &output) != PID_STATUS_OK)
{
    Error_Handler();
}
```

如果调用方已有可靠的误差变化率，例如从陀螺仪得到，可使用 `PID_CalculateWithDerivative()`。传入的是误差 `target_value - current_value` 的变化率，单位为“误差单位/秒”：

```c
if (PID_CalculateWithDerivative(
        &speed_config,
        measured_speed,
        target_speed,
        error_derivative,
        dt_seconds,
        &speed_history,
        &output) != PID_STATUS_OK)
{
    Error_Handler();
}
```

`dt_seconds` 必须是大于 0 的实际采样间隔（秒）。输出会被限制在配置范围内；积分值受积分限幅约束，并在输出饱和且积分继续推动饱和时暂停累积。参数无效时返回 `PID_STATUS_INVALID_ARGUMENT`，且本次调用不会更新历史值或输出；只有返回 `PID_STATUS_OK` 时才使用输出。
