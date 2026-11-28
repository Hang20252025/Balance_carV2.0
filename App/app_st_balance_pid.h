#ifndef APP_ST_BALANCE_PID_H
#define APP_ST_BALANCE_PID_H

#include "app_st_imu.h"

HAL_StatusTypeDef APP_ST_BALANCE_PID_Init(void);
HAL_StatusTypeDef APP_ST_BALANCE_PID_Update(
    const APP_ST_IMU_Data *imu_data,
    float target_pitch_deg,
    float *drive_output);
HAL_StatusTypeDef APP_ST_BALANCE_PID_Stop(void);
HAL_StatusTypeDef APP_ST_BALANCE_PID_SetKp(float kp);
HAL_StatusTypeDef APP_ST_BALANCE_PID_SetKi(float ki);
HAL_StatusTypeDef APP_ST_BALANCE_PID_SetKd(float kd);

#endif /* APP_ST_BALANCE_PID_H */
