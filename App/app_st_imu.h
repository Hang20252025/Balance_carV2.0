#ifndef APP_ST_IMU_H
#define APP_ST_IMU_H

#include <stdint.h>
#include "main.h"

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float gyro_x_deg_s;
    float gyro_y_deg_s;
    float gyro_z_deg_s;
} APP_ST_IMU_Data;

HAL_StatusTypeDef APP_ST_IMU_Init(uint32_t calibration_samples);
HAL_StatusTypeDef APP_ST_IMU_Update(void);
void APP_ST_IMU_GetData(APP_ST_IMU_Data *data);
float APP_ST_IMU_GetPitchAngle(void);
float APP_ST_IMU_GetRollAngle(void);

#endif /* APP_ST_IMU_H */
