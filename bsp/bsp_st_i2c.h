#ifndef BSP_ST_I2C_H
#define BSP_ST_I2C_H

#include <stddef.h>
#include <stdint.h>
#include "main.h"

#define BSP_ST_I2C_COUNT 2U
#define BSP_ST_I2C_TIMEOUT_MS 100U

/*
 * 将已配置的总线映射到 CubeMX 生成的 I2C 句柄。
 * 使用句柄时，请确保生成的 i2c.h 在本头文件之前包含。
 */
#define BSP_ST_I2C_1_HANDLE (&hi2c1)
#define BSP_ST_I2C_2_HANDLE (&hi2c2)

typedef enum
{
    BSP_ST_I2C_ID_1 = 1,
    BSP_ST_I2C_ID_2
} BSP_ST_I2C_Id; // I2C 总线编号

typedef enum
{
    BSP_ST_I2C_REGISTER_ADDRESS_8BIT = 1,
    BSP_ST_I2C_REGISTER_ADDRESS_16BIT = 2
} BSP_ST_I2C_RegisterAddressSize;// 寄存器地址大小

/* 设备地址使用未左移的 7 位 I2C 地址。 */
HAL_StatusTypeDef BSP_ST_I2C_Init(void);

HAL_StatusTypeDef BSP_ST_I2C_IsDeviceReady(
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    uint32_t trials  
);

HAL_StatusTypeDef BSP_ST_I2C_Write(// 写入数据
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    const uint8_t *data,
    uint16_t length);

HAL_StatusTypeDef BSP_ST_I2C_Read(
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    uint8_t *data,
    uint16_t length);

HAL_StatusTypeDef BSP_ST_I2C_WriteRegister(// 写入寄存器
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    uint16_t register_address,
    BSP_ST_I2C_RegisterAddressSize register_address_size,
    const uint8_t *data,
    uint16_t length);

HAL_StatusTypeDef BSP_ST_I2C_ReadRegister(
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    uint16_t register_address,
    BSP_ST_I2C_RegisterAddressSize register_address_size,
    uint8_t *data,
    uint16_t length);

#endif /* BSP_ST_I2C_H */