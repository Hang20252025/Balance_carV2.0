#ifndef APP_ST_DRIVE_H
#define APP_ST_DRIVE_H

#include "main.h"

HAL_StatusTypeDef APP_ST_DRIVE_Init(void);
HAL_StatusTypeDef APP_ST_DRIVE_Apply(float balance_output, float turn_output);
HAL_StatusTypeDef APP_ST_DRIVE_Stop(void);

#endif /* APP_ST_DRIVE_H */
