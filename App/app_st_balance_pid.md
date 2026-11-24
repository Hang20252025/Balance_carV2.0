# `app_st_balance_pid` 使用说明

`app_st_balance_pid` 使用 MPU6050 的俯仰角和 Y 轴角速度计算平衡控制量，并通过现有电机 BSP 输出双路 PWM。接口位于 [`app_st_balance_pid.h`](./app_st_balance_pid.h)，实现位于 [`app_st_balance_pid.c`](./app_st_balance_pid.c)。

## 接入方式

先完成 GPIO、定时器和 IMU 初始化，再初始化平衡模块。主循环中每次成功读取 IMU 后调用 PID 更新；传感器读取失败时立即停止电机：

```c
APP_ST_IMU_Data imu_data;

if (APP_ST_BALANCE_PID_Init() != HAL_OK)
{
    Error_Handler();
}

while (1)
{
    if (APP_ST_IMU_Update() == HAL_OK)
    {
        APP_ST_IMU_GetData(&imu_data);
        (void)APP_ST_BALANCE_PID_Update(&imu_data);
    }
    else if (APP_ST_BALANCE_PID_Stop() != HAL_OK)
    {
        Error_Handler();
    }

    HAL_Delay(5U);
}
```

PID 更新基于实际的 `HAL_GetTick()` 间隔；间隔超过 30 ms、俯仰角超过 ±30° 或传感器数据无效时会将 PWM 置零。需保持控制循环连续；同步 OLED 刷新会阻塞循环，不应放在运行中的控制循环里。

## 参数和方向调整

在 `app_st_balance_pid.c` 顶部调整目标角、`Kp`、`Ki`、`Kd`、最大 PWM 和最大倾角。当前 `Ki` 默认是 0，便于先用 PD 控制调试；实际参数需根据车体、供电和电机实测整定。

`APP_ST_BALANCE_CONTROL_REVERSE` 控制整个闭环输出的极性；若任一轮的正向与车体前进方向相反，分别调整 `APP_ST_BALANCE_MOTOR_1_REVERSE` 和 `APP_ST_BALANCE_MOTOR_2_REVERSE`。不同电机驱动器及安装方向可能需要不同配置。

## 安全调试

先将车轮架空，确认 IMU 俯仰方向、电机正反转和倾倒时电机响应方向正确，再低速落地测试。若车体一倾斜，电机反而继续加剧倾倒，立即断电并反转控制极性。默认参数只是起始值，不保证适配所有车体；达到最大倾角或控制周期异常时，模块会停 PWM。
