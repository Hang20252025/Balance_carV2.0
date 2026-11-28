#ifndef APP_ST_SPEED_H
#define APP_ST_SPEED_H

#include "main.h"
#include "pid_controller.h"

HAL_StatusTypeDef APP_ST_SPEED_Init(const PID_Config *config);
HAL_StatusTypeDef APP_ST_SPEED_Reset(void);
HAL_StatusTypeDef APP_ST_SPEED_Update(
    float target_speed,
    float measured_speed,
    float dt_seconds,
    float *target_pitch_deg);

#endif /* APP_ST_SPEED_H */
