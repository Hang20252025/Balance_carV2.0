# `app_vofa` 使用说明

`app_vofa` 通过 USART1 使用 VOFA+ FireWater 文本协议发送数据，并接收平衡 PID 参数。接口定义在 [`app_vofa.h`](./app_vofa.h)，实现位于 [`app_vofa.c`](./app_vofa.c)。

## 串口与初始化

当前 USART1 配置为 115200 波特率、8 数据位、无校验、1 停止位。`BSP_ST_UART_1_HANDLE` 已映射到 `huart1`。初始化 CubeMX 外设后调用 `APP_VOFA_Init()`；它会初始化 UART BSP 并启动中断接收。工程的主循环已调用初始化和处理接口，通常不需要重复接入。

若在其他程序中使用，调用顺序如下：

```c
MX_USART1_UART_Init();

if (APP_VOFA_Init() != HAL_OK)
{
    Error_Handler();
}

while (1)
{
    if (APP_VOFA_Process() != HAL_OK)
    {
        Error_Handler();
    }

    /* 应用逻辑 */
}
```

UART 中断必须保持启用；当前工程由 `USART1_IRQHandler()` 转调 HAL UART 中断处理，并在 USART1 MSP 初始化时配置 NVIC。

## 向上位机发送

`APP_VOFA_Send()` 接受通道数组和数量。每个数据项由通道名和值构成，输出是逗号分隔的 `name:value`，数值保留 3 位小数，末尾为 CRLF。例如：

```text
speed:1.250,distance:3.500\r\n
```

只发送速度：

```c
APP_VOFA_Channel channels[1] =
{
    {"speed", speed}
};

if (APP_VOFA_Send(channels, 1U) != HAL_OK)
{
    /* 处理发送错误 */
}
```

同时发送速度和距离：

```c
APP_VOFA_Channel channels[2] =
{
    {"speed", speed},
    {"distance", distance}
};

if (APP_VOFA_Send(channels, 2U) != HAL_OK)
{
    /* 处理发送错误 */
}
```

通道名不能为空，也不能包含逗号、冒号、方括号或换行符；数值必须是有限浮点数。单帧发送缓存为 256 字节，通道名和值的组合必须能放入同一帧。

在 VOFA+ 中选择与 USART1 相同的串口号和 115200、8-N-1 参数，并选择 FireWater 协议即可查看数据。

## 通信测试模块

[`app_vofa_test.c`](./app_vofa_test.c) 提供测试发送模块，主程序在每次 IMU 更新成功时调用它。模块每 50 ms 发送一帧，并交替发送单通道 `pitch` 与双通道 `pitch`、`gyro_y`，便于验证不同通道数量的 FireWater 数据接收。上位机仍可按下节格式发送 PID 参数，主循环中的 `APP_VOFA_Process()` 会持续解析。

## 从上位机调整 PID

上位机发送以方括号包裹的键值帧，冒号前是参数名，冒号后是数值：

```text
[kp:35.000]
[ki:0.100]
[kd:0.900]
```

`kp`、`ki`、`kd` 分别更新平衡 PID 的比例、积分和微分系数。参数接收后会用于后续 PID 更新；默认值为 `kp=35.0`、`ki=0.0`、`kd=0.9`。每次只能设置一个参数，未知参数、格式错误或非有限数值会被忽略，不会回复确认帧。

接收由 `APP_VOFA_Process()` 从 UART 环形缓冲区取出数据并解析。应在主循环或任务中定期调用；函数每次最多处理当前可读的 32 字节，未处理的数据留待下次调用。接收帧最长为 31 个字符（不含方括号），过长帧会被丢弃。若长时间不调用，UART 环形缓冲区可能溢出。

## 接口返回值

- `APP_VOFA_Init()`：UART 初始化或启动接收失败时返回非 `HAL_OK`。
- `APP_VOFA_Send()`：参数无效、数据超出发送缓存或 UART 发送失败时返回非 `HAL_OK`。
- `APP_VOFA_Process()`：UART 接收状态异常或 PID 参数更新失败时返回非 `HAL_OK`。

调用方应检查返回状态并按系统需求处理错误。
