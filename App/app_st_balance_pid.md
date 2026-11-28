# 平衡车 PID 控制模块

控制代码分为通用 PID 算法、各控制环和电机混控输出：

- [`pid_controller.h`](../Service/pid_controller.h)：不依赖硬件的 PID 参数、状态和计算接口。
- [`app_st_balance_pid.h`](./app_st_balance_pid.h)：平衡环，输入 IMU 数据和目标俯仰角，输出基础驱动量。
- [`app_st_speed.h`](./app_st_speed.h)：速度环，输入目标速度与测量速度，输出目标俯仰角。
- [`app_st_steering.h`](./app_st_steering.h)：转向环，输入目标/测量航向角，输出转向差速量。
- [`app_st_drive.h`](./app_st_drive.h)：将平衡输出和转向输出混控后交给电机 BSP。

各模块的完整 API 和调用示例分别见 [`PID 算法说明`](../Service/pid_controller.md)、[`速度环说明`](./app_st_speed.md)、[`转向环说明`](./app_st_steering.md) 和 [`电机混控说明`](./app_st_drive.md)。

每个 PID 控制环单独保存 `PID_History`，并在每次计算时将当前测量值、目标值、采样间隔、配置和历史值显式传给算法。`PID_Config` 中的限幅和增益应按对应控制量的单位设置。`derivative_filter_alpha` 范围为 0 到 1，越小滤波越强，1 表示不滤波。

## 当前主循环

项目已有 IMU 和电机接口，主程序当前只启用平衡环，目标俯仰角为 0，转向量为 0：

```c
APP_ST_IMU_Data imu_data;
float balance_output;
HAL_StatusTypeDef status;

if (APP_ST_DRIVE_Init() != HAL_OK ||
    APP_ST_BALANCE_PID_Init() != HAL_OK)
{
    Error_Handler();
}

while (1)
{
    if (APP_ST_IMU_Update() == HAL_OK)
    {
        APP_ST_IMU_GetData(&imu_data);
        status = APP_ST_BALANCE_PID_Update(&imu_data, 0.0f, &balance_output);

        if (status == HAL_OK)
        {
            if (APP_ST_DRIVE_Apply(balance_output, 0.0f) != HAL_OK)
            {
                Error_Handler();
            }
        }
        else if (status == HAL_BUSY)
        {
            (void)APP_ST_DRIVE_Stop();
        }
        else
        {
            if (APP_ST_DRIVE_Stop() != HAL_OK)
            {
                Error_Handler();
            }
            Error_Handler();
        }
    }
    else if (APP_ST_BALANCE_PID_Stop() != HAL_OK ||
             APP_ST_DRIVE_Stop() != HAL_OK)
    {
        Error_Handler();
    }

    HAL_Delay(5U);
}
```

IMU 读取失败时应停止电机。实时控制循环中不要同步刷新 OLED，以免 I2C 阻塞控制。

## 速度与转向环接口

速度环和转向环只处理输入数据并返回控制量，不读取传感器。速度反馈需要编码器等硬件提供；航向反馈需要陀螺仪积分或其他航向估计。当前工程尚未将这些传感器接入主循环，因此这两个环已提供独立接口，但暂不参与实际驱动。每次更新的 `dt_seconds` 需由调用方按真实采样周期计算。

```c
const PID_Config speed_config = {
    1.0f, 0.0f, 0.0f, -5.0f, 5.0f, -2.0f, 2.0f, 1.0f
};
const PID_Config steering_config = {
    1.0f, 0.0f, 0.0f, -250.0f, 250.0f, -20.0f, 20.0f, 1.0f
};
float target_pitch_deg;
float turn_output;

if (APP_ST_SPEED_Init(&speed_config) != HAL_OK ||
    APP_ST_STEERING_Init(&steering_config) != HAL_OK)
{
    Error_Handler();
}

if (APP_ST_SPEED_Update(
        target_speed, measured_speed, dt_seconds, &target_pitch_deg) != HAL_OK ||
    APP_ST_STEERING_Update(
        target_yaw_deg, measured_yaw_deg, dt_seconds, &turn_output) != HAL_OK)
{
    Error_Handler();
}
```

串级控制时，将速度环的 `target_pitch_deg` 传给平衡环；再把平衡环输出和转向环输出传给 `APP_ST_DRIVE_Apply()`。驱动模块按 `motor1 = balance + turn`、`motor2 = balance - turn` 混合，并将结果限幅到 PWM 范围。示例 PID 参数仅展示接口格式，不是可直接上车的整定值；速度、航向、角度、PWM 的量纲和符号需与实际硬件校准一致。

## 平衡环保护与调参

平衡环按实际 `HAL_GetTick()` 间隔计算，控制间隔超过 30 ms、俯仰角超过 ±30° 或 IMU 数值无效时返回 `HAL_BUSY` 并输出零。超过倾角阈值会锁定停机，需重新初始化平衡模块后才会恢复；IMU 数值或控制周期异常停机后，可在数据恢复时继续运行。其他无效参数或未初始化时返回 `HAL_ERROR`。

平衡环的初始 `Kp/Ki/Kd` 在 `app_st_balance_pid.c` 中配置，当前 `Ki` 为 0；**实际参数需根据车体、供电和电机实测整定。**驱动模块的两个电机方向极性在 `app_st_drive.c` 中分别配置。

首次上车前先架空车轮确认方向。若车体倾斜后电机响应反而加剧倾倒，立即断电并检查 IMU 轴向、控制输出极性和左右电机方向。
