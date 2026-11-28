#ifndef APP_ST_STEERING_H
#define APP_ST_STEERING_H

#include "main.h"
#include "pid_controller.h"

HAL_StatusTypeDef APP_ST_STEERING_Init(const PID_Config *config);
HAL_StatusTypeDef APP_ST_STEERING_Reset(void);
HAL_StatusTypeDef APP_ST_STEERING_Update(
    float target_yaw_deg,
    float measured_yaw_deg,
    float dt_seconds,
    float *turn_output);

#endif /* APP_ST_STEERING_H */
