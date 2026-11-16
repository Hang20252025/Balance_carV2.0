# `bsp_st_uart` 使用说明

`bsp_st_uart` 为最多四个 UART 提供统一的发送、非阻塞接收和接收统计接口。UART 编号、接收缓冲区大小和 HAL 句柄映射定义在 [`bsp_st_uart.h`](./bsp_st_uart.h)，实现位于 [`bsp_st_uart.c`](./bsp_st_uart.c)。

## 配置 UART 句柄

头文件中的四个句柄宏默认都是 `NULL`，因此默认没有 UART 可用；至少映射一个已由 CubeMX 生成的 UART 句柄后才能初始化。例如，项目生成了 `huart1` 和 `huart2` 时，可在 `bsp_st_uart.h` 中配置：

```c
#define BSP_ST_UART_1_HANDLE (&huart1)
#define BSP_ST_UART_2_HANDLE (&huart2)
#define BSP_ST_UART_3_HANDLE NULL
#define BSP_ST_UART_4_HANDLE NULL
```

确保声明 `huart1`、`huart2` 的生成头文件（通常为 `usart.h`）在使用这些宏的编译单元中可见。具体来说，若在头文件中映射到 `huartx`，应在 `bsp_st_uart.c` 中把 `#include "usart.h"` 放在 `#include "bsp_st_uart.h"` 之前。每个宏必须映射到不同的句柄；未配置的编号不可用于发送或接收。

`BSP_ST_UART_RX_BUFFER_SIZE` 默认是 256。环形缓冲区会保留一个空位用于区分“满”和“空”，所以每个 UART 最多可缓存 255 个字节。若修改该宏，需确保其值至少为 2。

## 初始化

先初始化 CubeMX 生成的 UART 外设，再调用 BSP 初始化。对应 UART 的 NVIC 中断也必须已配置并启用。

```c
#include "usart.h"
#include "bsp_st_uart.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    if (BSP_ST_UART_Init() != HAL_OK)
    {
        Error_Handler();
    }

    while (1)
    {
        /* 应用逻辑 */
    }
}
```

只调用实际映射了句柄的 `MX_USARTx_UART_Init()`。`BSP_ST_UART_Init()` 会检查映射和重复句柄，并为已配置 UART 启动单字节中断接收；没有配置任何 UART、句柄无效或启动接收失败时会返回错误。

> **当前实现注意事项：** `bsp_st_uart.c` 的 `BSP_ST_UART_Init()` 中，跳过未配置/已启动 UART 的 `if` 语句后有一个额外的独立 `{ continue; }` 代码块。它会无条件跳过启动接收的代码，导致函数可能返回 `HAL_OK`，但接收实际上没有启动。使用接收功能前需移除该多余代码块。

## 发送数据

`BSP_ST_UART_Send()` 是阻塞发送，超时时间由 `BSP_ST_UART_TX_TIMEOUT_MS` 控制（默认 1000 ms）。`length` 是要发送的字节数，必须不大于 `data` 指向的有效数据长度。

```c
static const uint8_t message[] = "Hello\r\n";

if (BSP_ST_UART_Send(
        BSP_ST_UART_ID_1,
        message,
        (uint16_t)(sizeof(message) - 1U)) != HAL_OK)
{
    Error_Handler();
}
```

上例不发送字符串末尾的 `'\0'`。发送二进制数据时，`length` 应设为实际有效字节数。长度为 0 时可以传入 `NULL` 数据指针。

## 接收数据

接收由中断在后台逐字节进行，字节先存入 BSP 的环形缓冲区。应用调用 `BSP_ST_UART_Receive()` 时，函数将当前可取出的数据复制到应用缓冲区；该函数不会等待新数据到达。

```c
uint8_t rx_buffer[64];
uint16_t received_size;
HAL_StatusTypeDef status;

status = BSP_ST_UART_Receive(
    BSP_ST_UART_ID_1,
    rx_buffer,
    sizeof(rx_buffer),
    &received_size);

if (received_size > 0U)
{
    /* 处理 rx_buffer[0] 到 rx_buffer[received_size - 1] */
}

if (status != HAL_OK)
{
    /* 处理 UART 未配置、未初始化或接收未处于挂起状态等情况 */
}
```

`received_size` 返回本次实际取出的字节数，可能为 0。返回 `HAL_OK` 表示调用结束时接收已挂起，不代表本次一定读到数据；反过来，如果接收未挂起，函数可能已经复制了一些数据再返回 `HAL_ERROR`，因此应先检查 `received_size` 并处理已取出的字节。

若要一次读出最多缓存的数据，应用缓冲区至少要有 255 字节，并将其实际容量传给 `buffer_size`。该容量必须不大于 `buffer` 可写的字节数。

## 读取接收统计

```c
BSP_ST_UART_RxStats stats;

if (BSP_ST_UART_GetRxStats(BSP_ST_UART_ID_1, &stats) == HAL_OK)
{
    /* 使用 stats.dropped_bytes、stats.uart_errors、
       stats.rx_rearm_errors */
}
```

- `dropped_bytes`：环形缓冲区满时丢弃的字节数。
- b  踩踩踩踩踩踩踩踩踩踩踩踩踩踩踩踩踩踩从bn

## 错误处理与集成注意事项

- `HAL_OK` 表示接口操作成功；无效编号、未初始化、未配置句柄或无效参数等情况会返回 `HAL_ERROR`。发送还可能返回 HAL 的超时或忙碌状态。
- 本模块实现了 `HAL_UART_RxCpltCallback()` 和 `HAL_UART_ErrorCallback()`。如果应用工程中已经定义了同名 HAL 回调，应将本模块逻辑合并到现有回调中，避免重复定义或覆盖回调逻辑。
- `BSP_ST_UART_Receive()` 应在主循环或任务中定期调用；若应用处理速度长期低于接收速度，环形缓冲区可能填满并增加 `dropped_bytes`。
