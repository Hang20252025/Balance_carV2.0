#ifndef APP_VOFA_H
#define APP_VOFA_H

#include <stddef.h>
#include "main.h"

typedef struct
{
    const char *name;
    float value;
} APP_VOFA_Channel;

HAL_StatusTypeDef APP_VOFA_Init(void);
/* Sends comma-separated name:value channels followed by CRLF. */
HAL_StatusTypeDef APP_VOFA_Send(
    const APP_VOFA_Channel *channels,
    size_t channel_count);
HAL_StatusTypeDef APP_VOFA_Process(void);

#endif /* APP_VOFA_H */
