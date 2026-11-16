#include "bsp_st_uart.h"

//
typedef struct
{
    uint8_t rx_byte;
    uint8_t rx_buffer[BSP_ST_UART_RX_BUFFER_SIZE];
    volatile uint16_t head;//环形缓冲区的头索引
    volatile uint16_t tail;//环形缓冲区的尾索引
    volatile uint32_t dropped_bytes;//丢弃的字节数
    volatile uint32_t uart_errors;//UART错误数
    volatile uint32_t rx_rearm_errors;//重新启动接收中断错误数
    volatile uint8_t rx_armed;//接收中断是否已经启动
} BSP_ST_UART_State;

//这是一个句柄的数组
static UART_HandleTypeDef *const s_uart_handles[BSP_ST_UART_COUNT] =
{
    BSP_ST_UART_1_HANDLE,
    BSP_ST_UART_2_HANDLE,
    BSP_ST_UART_3_HANDLE,
    BSP_ST_UART_4_HANDLE
};


static BSP_ST_UART_State s_uart_states[BSP_ST_UART_COUNT];
static uint8_t s_uart_initialized;

//进行检查并获取索引
static uint8_t BSP_ST_UART_GetIndex(BSP_ST_UART_Id uart_id, uint32_t *index)
{
    if ((index == NULL) ||
        (uart_id < BSP_ST_UART_ID_1) ||
        (uart_id > BSP_ST_UART_ID_4))
    {
        return 0U;
    }

    *index = (uint32_t)uart_id - 1U;
    return 1U;
}

//查找当前句柄对应是哪个索引
static uint32_t BSP_ST_UART_FindHandle(const UART_HandleTypeDef *handle)
{
    uint32_t i;

    if (handle == NULL)
    {
        return BSP_ST_UART_COUNT;
    }

    for (i = 0U; i < BSP_ST_UART_COUNT; i++)
    {
        if (s_uart_handles[i] == handle)
        {
            return i;
        }
    }

    return BSP_ST_UART_COUNT;
}


static HAL_StatusTypeDef BSP_ST_UART_ArmReceive(uint32_t index)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Receive_IT(
        s_uart_handles[index],
        &s_uart_states[index].rx_byte,
        1U);

    if (status == HAL_OK)
    {
        s_uart_states[index].rx_armed = 1U;
    }
    else
    {
        s_uart_states[index].rx_armed = 0U;
        s_uart_states[index].rx_rearm_errors++;
    }

    return status;
}

HAL_StatusTypeDef BSP_ST_UART_Init(void)
{
    uint32_t i;
    uint32_t j;
    uint32_t configured_count = 0U;

    for (i = 0U; i < BSP_ST_UART_COUNT; i++)
    {
        if (s_uart_handles[i] == NULL)
        {
            continue;
        }
        
        //检查到底是否确实有这个硬件的存在
        if (s_uart_handles[i]->Instance == NULL)
        {
            return HAL_ERROR;
        }

        configured_count++;
        
        //检查是否有重复的句柄，比如说宏定义的时候可能多个宏定义写成了同一个句柄
        for (j = i + 1U; j < BSP_ST_UART_COUNT; j++)
        {
            if (s_uart_handles[i] == s_uart_handles[j])
            {
                return HAL_ERROR;
            }
        }
    }
    //如果没有配置任何一个串口，就返回错误
    if (configured_count == 0U)
    {
        return HAL_ERROR;
    }
    //初始化状态结构体
    for (i = 0U; i < BSP_ST_UART_COUNT; i++)
    {
        HAL_StatusTypeDef status;

        if ((s_uart_handles[i] == NULL) || (s_uart_states[i].rx_armed != 0U))//如果没有配置这个串口或者已经启动了接收，就跳过
        {
            continue;
        }
        {
            continue;
        }

        status = BSP_ST_UART_ArmReceive(i);//启动接收中断
        if (status != HAL_OK)
        {
            return status;
        }
    }

    s_uart_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef BSP_ST_UART_Send(
    BSP_ST_UART_Id uart_id,
    const uint8_t *data,
    uint16_t length)
{
    uint32_t index;

    if (!BSP_ST_UART_GetIndex(uart_id, &index) ||
        (s_uart_initialized == 0U) ||
        (s_uart_handles[index] == NULL) ||
        ((data == NULL) && (length != 0U)))
    {
        return HAL_ERROR;
    }

    if (length == 0U)
    {
        return HAL_OK;
    }

    /* HAL's transmit API takes a non-const pointer but does not modify Tx data. */
    return HAL_UART_Transmit(
        s_uart_handles[index],
        (uint8_t *)data,
        length,
        BSP_ST_UART_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef BSP_ST_UART_Receive(
    BSP_ST_UART_Id uart_id,
    uint8_t *buffer,
    uint16_t buffer_size,
    uint16_t *received_size)
{
    uint32_t index;
    uint16_t received = 0U;
    BSP_ST_UART_State *state;

    if (received_size == NULL)
    {
        return HAL_ERROR;
    }
    *received_size = 0U;

    if (!BSP_ST_UART_GetIndex(uart_id, &index) ||
        (s_uart_initialized == 0U) ||
        (s_uart_handles[index] == NULL) ||
        ((buffer == NULL) && (buffer_size != 0U)))
    {
        return HAL_ERROR;
    }

    state = &s_uart_states[index];
    while ((state->tail != state->head) && (received < buffer_size))
    {
        buffer[received++] = state->rx_buffer[state->tail];
        state->tail = (uint16_t)((state->tail + 1U) % BSP_ST_UART_RX_BUFFER_SIZE);
    }

    *received_size = received;
    return (state->rx_armed != 0U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef BSP_ST_UART_GetRxStats(
    BSP_ST_UART_Id uart_id,
    BSP_ST_UART_RxStats *stats)
{
    uint32_t index;

    if ((stats == NULL) ||
        !BSP_ST_UART_GetIndex(uart_id, &index) ||
        (s_uart_handles[index] == NULL))
    {
        return HAL_ERROR;
    }

    stats->dropped_bytes = s_uart_states[index].dropped_bytes;
    stats->uart_errors = s_uart_states[index].uart_errors;
    stats->rx_rearm_errors = s_uart_states[index].rx_rearm_errors;

    return HAL_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t index = BSP_ST_UART_FindHandle(huart);
    BSP_ST_UART_State *state;
    uint16_t next_head;

    if (index >= BSP_ST_UART_COUNT)
    {
        return;
    }

    state = &s_uart_states[index];
    state->rx_armed = 0U;
    next_head = (uint16_t)((state->head + 1U) % BSP_ST_UART_RX_BUFFER_SIZE);//形成环形索引

    if (next_head == state->tail)
    {
        state->dropped_bytes++;
    }
    else
    {
        state->rx_buffer[state->head] = state->rx_byte;
        state->head = next_head;
    }

    (void)BSP_ST_UART_ArmReceive(index);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t index = BSP_ST_UART_FindHandle(huart);

    if (index >= BSP_ST_UART_COUNT)
    {
        return;
    }

    s_uart_states[index].uart_errors++;

    if (huart->RxState == HAL_UART_STATE_READY)
    {
        s_uart_states[index].rx_armed = 0U;
        (void)BSP_ST_UART_ArmReceive(index);
    }
}