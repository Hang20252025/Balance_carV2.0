#ifndef BSP_ST_UART_H
#define BSP_ST_UART_H

#include <stddef.h>
#include <stdint.h>
#include "main.h"

#define BSP_ST_UART_COUNT 4U
#define BSP_ST_UART_RX_BUFFER_SIZE 256U
#define BSP_ST_UART_TX_TIMEOUT_MS 1000U

#if BSP_ST_UART_RX_BUFFER_SIZE < 2U
#error BSP_ST_UART_RX_BUFFER_SIZE must be at least 2
#endif

/*
 * Map configured CubeMX UART handles here; unconfigured slots stay NULL.
 * When mapping a handle, include the generated usart.h before this header.
 */
#define BSP_ST_UART_1_HANDLE NULL
#define BSP_ST_UART_2_HANDLE NULL
#define BSP_ST_UART_3_HANDLE NULL
#define BSP_ST_UART_4_HANDLE NULL

//枚举类型，后面的赋值自动递增
typedef enum
{
    BSP_ST_UART_ID_1 = 1,
    BSP_ST_UART_ID_2,
    BSP_ST_UART_ID_3,
    BSP_ST_UART_ID_4
} BSP_ST_UART_Id;

typedef struct
{
    uint32_t dropped_bytes;
    uint32_t uart_errors;
    uint32_t rx_rearm_errors;
} BSP_ST_UART_RxStats;

/* Call after CubeMX has initialized every mapped UART and its NVIC interrupt. */
HAL_StatusTypeDef BSP_ST_UART_Init(void);

/* Blocking transmit with BSP_ST_UART_TX_TIMEOUT_MS timeout. */
HAL_StatusTypeDef BSP_ST_UART_Send(
    BSP_ST_UART_Id uart_id,
    const uint8_t *data,
    uint16_t length);

/* Nonblocking read of currently buffered bytes; an empty queue returns size 0. */
HAL_StatusTypeDef BSP_ST_UART_Receive(
    BSP_ST_UART_Id uart_id,
    uint8_t *buffer,//接收数据的缓冲区
    uint16_t buffer_size,
    uint16_t *received_size//收到数据的大小
);
HAL_StatusTypeDef BSP_ST_UART_GetRxStats(
    BSP_ST_UART_Id uart_id,
    BSP_ST_UART_RxStats *stats);

#endif /* BSP_ST_UART_H */