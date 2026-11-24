#ifndef APP_ST_BALANCE_PID_H
#define APP_ST_BALANCE_PID_H

#include "app_st_imu.h"

HAL_StatusTypeDef APP_ST_BALANCE_PID_Init(void);
HAL_StatusTypeDef APP_ST_BALANCE_PID_Update(const APP_ST_IMU_Data *imu_data);
HAL_StatusTypeDef APP_ST_BALANCE_PID_Stop(void);

#endif /* APP_ST_BALANCE_PID_H */
