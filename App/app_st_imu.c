#include "app_st_imu.h"

#include <math.h>
#include "bsp_st_i2c.h"

#define APP_ST_IMU_ADDRESS                  0x68U
#define APP_ST_IMU_REG_WHO_AM_I             0x75U
#define APP_ST_IMU_WHO_AM_I_MPU6050         0x68U
#define APP_ST_IMU_REG_SMPLRT_DIV           0x19U
#define APP_ST_IMU_REG_CONFIG               0x1AU
#define APP_ST_IMU_REG_GYRO_CONFIG          0x1BU
#define APP_ST_IMU_REG_ACCEL_CONFIG         0x1CU
#define APP_ST_IMU_REG_ACCEL_XOUT_H         0x3BU
#define APP_ST_IMU_REG_PWR_MGMT_1           0x6BU

#define APP_ST_IMU_ACCEL_SCALE              16384.0f
#define APP_ST_IMU_GYRO_SCALE               131.0f
#define APP_ST_IMU_FILTER_COEFFICIENT       0.98f
#define APP_ST_IMU_DEFAULT_UPDATE_PERIOD_MS 5U

static APP_ST_IMU_Data s_imu_data;
static float s_gyro_x_offset;
static float s_gyro_y_offset;
static float s_gyro_z_offset;
static uint32_t s_last_update_tick;
static uint8_t s_imu_initialized;

static HAL_StatusTypeDef APP_ST_IMU_WriteRegister(
    uint8_t register_address,
    uint8_t value)
{
    return BSP_ST_I2C_WriteRegister(
        BSP_ST_I2C_ID_2,
        APP_ST_IMU_ADDRESS,
        register_address,
        BSP_ST_I2C_REGISTER_ADDRESS_8BIT,
        &value,
        1U);
}

static HAL_StatusTypeDef APP_ST_IMU_ReadRawData(
    int16_t *accel_x,
    int16_t *accel_y,
    int16_t *accel_z,
    int16_t *gyro_x,
    int16_t *gyro_y,
    int16_t *gyro_z)
{
    uint8_t raw_data[14];

    if ((accel_x == NULL) ||
        (accel_y == NULL) ||
        (accel_z == NULL) ||
        (gyro_x == NULL) ||
        (gyro_y == NULL) ||
        (gyro_z == NULL))
    {
        return HAL_ERROR;
    }

    if (BSP_ST_I2C_ReadRegister(
            BSP_ST_I2C_ID_2,
            APP_ST_IMU_ADDRESS,
            APP_ST_IMU_REG_ACCEL_XOUT_H,
            BSP_ST_I2C_REGISTER_ADDRESS_8BIT,
            raw_data,
            sizeof(raw_data)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *accel_x = (int16_t)((uint16_t)raw_data[0] << 8U) | raw_data[1];
    *accel_y = (int16_t)((uint16_t)raw_data[2] << 8U) | raw_data[3];
    *accel_z = (int16_t)((uint16_t)raw_data[4] << 8U) | raw_data[5];
    *gyro_x = (int16_t)((uint16_t)raw_data[8] << 8U) | raw_data[9];
    *gyro_y = (int16_t)((uint16_t)raw_data[10] << 8U) | raw_data[11];
    *gyro_z = (int16_t)((uint16_t)raw_data[12] << 8U) | raw_data[13];

    return HAL_OK;
}

static float APP_ST_IMU_MeasureComplementAngle(
    float accel_x,
    float accel_y,
    float accel_z)
{
    float norm;

    norm = sqrtf(
        (accel_y * accel_y) +
        (accel_z * accel_z));

    if (norm < 0.0001f)
    {
        return s_imu_data.pitch_deg;
    }

    return -atan2f(accel_x, norm) * 57.2957795f;
}

HAL_StatusTypeDef APP_ST_IMU_Init(uint32_t calibration_samples)
{
    uint8_t who_am_i = 0U;
    uint32_t i;
    float gyro_x_sum = 0.0f;
    float gyro_y_sum = 0.0f;
    float gyro_z_sum = 0.0f;

    if (BSP_ST_I2C_ReadRegister(
            BSP_ST_I2C_ID_2,
            APP_ST_IMU_ADDRESS,
            APP_ST_IMU_REG_WHO_AM_I,
            BSP_ST_I2C_REGISTER_ADDRESS_8BIT,
            &who_am_i,
            1U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (who_am_i != APP_ST_IMU_WHO_AM_I_MPU6050)
    {
        return HAL_ERROR;
    }

    if ((APP_ST_IMU_WriteRegister(APP_ST_IMU_REG_PWR_MGMT_1, 0x00U) != HAL_OK) ||
        (APP_ST_IMU_WriteRegister(APP_ST_IMU_REG_SMPLRT_DIV, 0x07U) != HAL_OK) ||
        (APP_ST_IMU_WriteRegister(APP_ST_IMU_REG_CONFIG, 0x03U) != HAL_OK) ||
        (APP_ST_IMU_WriteRegister(APP_ST_IMU_REG_GYRO_CONFIG, 0x00U) != HAL_OK) ||
        (APP_ST_IMU_WriteRegister(APP_ST_IMU_REG_ACCEL_CONFIG, 0x00U) != HAL_OK))
    {
        return HAL_ERROR;
    }

    HAL_Delay(30U);

    s_gyro_x_offset = 0.0f;
    s_gyro_y_offset = 0.0f;
    s_gyro_z_offset = 0.0f;

    if (calibration_samples == 0U)
    {
        calibration_samples = 400U;
    }

    for (i = 0U; i < calibration_samples; i++)
    {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;

        if (APP_ST_IMU_ReadRawData(
                &accel_x,
                &accel_y,
                &accel_z,
                &gyro_x,
                &gyro_y,
                &gyro_z) != HAL_OK)
        {
            return HAL_ERROR;
        }

        gyro_x_sum += (float)gyro_x / APP_ST_IMU_GYRO_SCALE;
        gyro_y_sum += (float)gyro_y / APP_ST_IMU_GYRO_SCALE;
        gyro_z_sum += (float)gyro_z / APP_ST_IMU_GYRO_SCALE;
        HAL_Delay(APP_ST_IMU_DEFAULT_UPDATE_PERIOD_MS);
    }

    s_gyro_x_offset = gyro_x_sum / (float)calibration_samples;
    s_gyro_y_offset = gyro_y_sum / (float)calibration_samples;
    s_gyro_z_offset = gyro_z_sum / (float)calibration_samples;

    s_imu_data.roll_deg = 0.0f;
    s_imu_data.pitch_deg = 0.0f;
    s_imu_data.gyro_x_deg_s = 0.0f;
    s_imu_data.gyro_y_deg_s = 0.0f;
    s_imu_data.gyro_z_deg_s = 0.0f;
    s_last_update_tick = HAL_GetTick();
    s_imu_initialized = 1U;

    return HAL_OK;
}

HAL_StatusTypeDef APP_ST_IMU_Update(void)
{
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_pitch_deg;
    float accel_roll_deg;
    float dt_seconds;
    uint32_t now_tick;

    if (s_imu_initialized == 0U)
    {
        return HAL_ERROR;
    }

    if (APP_ST_IMU_ReadRawData(
            &raw_accel_x,
            &raw_accel_y,
            &raw_accel_z,
            &raw_gyro_x,
            &raw_gyro_y,
            &raw_gyro_z) != HAL_OK)
    {
        return HAL_ERROR;
    }

    accel_x = (float)raw_accel_x / APP_ST_IMU_ACCEL_SCALE;
    accel_y = (float)raw_accel_y / APP_ST_IMU_ACCEL_SCALE;
    accel_z = (float)raw_accel_z / APP_ST_IMU_ACCEL_SCALE;

    gyro_x = ((float)raw_gyro_x / APP_ST_IMU_GYRO_SCALE) - s_gyro_x_offset;
    gyro_y = ((float)raw_gyro_y / APP_ST_IMU_GYRO_SCALE) - s_gyro_y_offset;
    gyro_z = ((float)raw_gyro_z / APP_ST_IMU_GYRO_SCALE) - s_gyro_z_offset;

    now_tick = HAL_GetTick();
    dt_seconds = (float)(now_tick - s_last_update_tick) * 0.001f;
    s_last_update_tick = now_tick;

    if ((dt_seconds <= 0.0001f) ||
        (dt_seconds > 0.100f))
    {
        dt_seconds = (float)APP_ST_IMU_DEFAULT_UPDATE_PERIOD_MS * 0.001f;
    }

    accel_pitch_deg = APP_ST_IMU_MeasureComplementAngle(
        accel_x,
        accel_y,
        accel_z);

    accel_roll_deg = atan2f(
        accel_y,
        accel_z) * 57.2957795f;

    s_imu_data.pitch_deg =
        APP_ST_IMU_FILTER_COEFFICIENT *
            (s_imu_data.pitch_deg + gyro_y * dt_seconds) +
        (1.0f - APP_ST_IMU_FILTER_COEFFICIENT) *
            accel_pitch_deg;

    s_imu_data.roll_deg =
        APP_ST_IMU_FILTER_COEFFICIENT *
            (s_imu_data.roll_deg + gyro_x * dt_seconds) +
        (1.0f - APP_ST_IMU_FILTER_COEFFICIENT) *
            accel_roll_deg;

    s_imu_data.gyro_x_deg_s = gyro_x;
    s_imu_data.gyro_y_deg_s = gyro_y;
    s_imu_data.gyro_z_deg_s = gyro_z;

    return HAL_OK;
}

void APP_ST_IMU_GetData(APP_ST_IMU_Data *data)
{
    if (data != NULL)
    {
        *data = s_imu_data;
    }
}

float APP_ST_IMU_GetPitchAngle(void)
{
    return s_imu_data.pitch_deg;
}

float APP_ST_IMU_GetRollAngle(void)
{
    return s_imu_data.roll_deg;
}
