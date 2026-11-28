#include "app_vofa.h"

#include "app_st_balance_pid.h"
#include "bsp_st_uart.h"

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_VOFA_TX_BUFFER_SIZE       256U
#define APP_VOFA_RX_BUFFER_SIZE       32U
#define APP_VOFA_RX_CHUNK_SIZE        32U

static char s_rx_frame[APP_VOFA_RX_BUFFER_SIZE];
static uint8_t s_rx_frame_length;
static uint8_t s_rx_frame_active;
static uint8_t s_rx_frame_overflow;

static uint8_t APP_VOFA_IsFinite(float value)
{
    return (value == value) &&
           (value <= FLT_MAX) &&
           (value >= -FLT_MAX);
}

static uint8_t APP_VOFA_IsValidChannelName(const char *name)
{
    if ((name == NULL) || (*name == '\0'))
    {
        return 0U;
    }

    while (*name != '\0')
    {
        if ((*name == ',') || (*name == ':') ||
            (*name == '[') || (*name == ']') ||
            (*name == '\r') || (*name == '\n'))
        {
            return 0U;
        }
        name++;
    }

    return 1U;
}

static HAL_StatusTypeDef APP_VOFA_ParseFrame(void)
{
    char *separator;
    char *value_text;
    char *value_end;
    float value;

    s_rx_frame[s_rx_frame_length] = '\0';
    separator = strchr(s_rx_frame, ':');
    if (separator == NULL)
    {
        return HAL_OK;
    }

    *separator = '\0';
    value_text = separator + 1;
    value = strtof(value_text, &value_end);
    if ((value_end == value_text) || (*value_end != '\0') ||
        !APP_VOFA_IsFinite(value))
    {
        return HAL_OK;
    }

    if (strcmp(s_rx_frame, "kp") == 0)
    {
        return APP_ST_BALANCE_PID_SetKp(value);
    }
    if (strcmp(s_rx_frame, "ki") == 0)
    {
        return APP_ST_BALANCE_PID_SetKi(value);
    }
    if (strcmp(s_rx_frame, "kd") == 0)
    {
        return APP_ST_BALANCE_PID_SetKd(value);
    }

    return HAL_OK;
}

static HAL_StatusTypeDef APP_VOFA_ProcessByte(uint8_t byte)
{
    HAL_StatusTypeDef status;

    if (byte == (uint8_t)'[')
    {
        s_rx_frame_length = 0U;//重置长度
        s_rx_frame_active = 1U;//标记帧开始
        s_rx_frame_overflow = 0U;//重置溢出标志
        return HAL_OK;
    }

    if (s_rx_frame_active == 0U)
    {
        return HAL_OK;
    }

    if (byte == (uint8_t)']')
    {
        if (s_rx_frame_overflow != 0U)
        {
            s_rx_frame_active = 0U;
            return HAL_OK;
        }

        status = APP_VOFA_ParseFrame();
        s_rx_frame_active = 0U;
        return status;
    }

    if (s_rx_frame_overflow != 0U)
    {
        return HAL_OK;
    }

    if (s_rx_frame_length >= (APP_VOFA_RX_BUFFER_SIZE - 1U))
    {
        s_rx_frame_overflow = 1U;
        return HAL_OK;
    }

    s_rx_frame[s_rx_frame_length++] = (char)byte;
    return HAL_OK;
}

HAL_StatusTypeDef APP_VOFA_Init(void)
{
    s_rx_frame_length = 0U;
    s_rx_frame_active = 0U;
    s_rx_frame_overflow = 0U;

    return BSP_ST_UART_Init();
}

HAL_StatusTypeDef APP_VOFA_Send(
    const APP_VOFA_Channel *channels,
    size_t channel_count)
{
    char tx_buffer[APP_VOFA_TX_BUFFER_SIZE];
    size_t used = 0U;
    size_t i;
    int written;

    if ((channels == NULL) || (channel_count == 0U))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < channel_count; i++)
    {
        if (!APP_VOFA_IsValidChannelName(channels[i].name) ||
            !APP_VOFA_IsFinite(channels[i].value))
        {
            return HAL_ERROR;
        }

        written = snprintf(
            &tx_buffer[used],
            sizeof(tx_buffer) - used,
            "%s%s:%.3f",
            (i == 0U) ? "" : ",",
            channels[i].name,
            (double)channels[i].value);
        if ((written < 0) || ((size_t)written >= (sizeof(tx_buffer) - used)))
        {
            return HAL_ERROR;
        }
        used += (size_t)written;
    }

    if ((sizeof(tx_buffer) - used) < 3U)
    {
        return HAL_ERROR;
    }
    tx_buffer[used++] = '\r';
    tx_buffer[used++] = '\n';
    tx_buffer[used] = '\0';

    return BSP_ST_UART_Send(
        BSP_ST_UART_ID_1,
        (const uint8_t *)tx_buffer,
        (uint16_t)used);
}

HAL_StatusTypeDef APP_VOFA_Process(void)
{
    uint8_t rx_buffer[APP_VOFA_RX_CHUNK_SIZE];
    uint16_t received_size;
    uint16_t i;
    HAL_StatusTypeDef status;

    status = BSP_ST_UART_Receive(
        BSP_ST_UART_ID_1,
        rx_buffer,
        sizeof(rx_buffer),
        &received_size);
    if (status != HAL_OK)
    {
        return status;
    }

    for (i = 0U; i < received_size; i++)
    {
        status = APP_VOFA_ProcessByte(rx_buffer[i]);
        if (status != HAL_OK)
        {
            return status;
        }
    }

    return HAL_OK;
}
