#include "i2c.h"
#include "bsp_st_i2c.h"

static I2C_HandleTypeDef *const s_i2c_handles[BSP_ST_I2C_COUNT] =
{
    BSP_ST_I2C_1_HANDLE,
    BSP_ST_I2C_2_HANDLE
};

static uint8_t s_i2c_initialized;

static uint8_t BSP_ST_I2C_GetIndex(BSP_ST_I2C_Id i2c_id, uint32_t *index)
{
    if ((index == NULL) ||
        (i2c_id < BSP_ST_I2C_ID_1) ||
        (i2c_id > BSP_ST_I2C_ID_2))
    {
        return 0U;
    }

    *index = (uint32_t)i2c_id - 1U;
    return 1U;
}

static uint8_t BSP_ST_I2C_IsValidDeviceAddress(uint8_t device_address)
{
    return (device_address > 0U) && (device_address <= 0x7FU);
}

static HAL_StatusTypeDef BSP_ST_I2C_GetHandle(
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    I2C_HandleTypeDef **handle)
{
    uint32_t index;

    if ((handle == NULL) ||
        !BSP_ST_I2C_GetIndex(i2c_id, &index) ||
        (s_i2c_initialized == 0U) ||
        (s_i2c_handles[index] == NULL) ||
        (s_i2c_handles[index]->Instance == NULL) ||
        !BSP_ST_I2C_IsValidDeviceAddress(device_address))
    {
        return HAL_ERROR;
    }

    *handle = s_i2c_handles[index];
    return HAL_OK;
}

static uint8_t BSP_ST_I2C_GetHalRegisterAddressSize(
    BSP_ST_I2C_RegisterAddressSize register_address_size,
    uint16_t register_address,
    uint16_t *hal_register_address_size)
{
    if (hal_register_address_size == NULL)
    {
        return 0U;
    }

    if (register_address_size == BSP_ST_I2C_REGISTER_ADDRESS_8BIT)
    {
        if (register_address > 0x00FFU)
        {
            return 0U;
        }
        *hal_register_address_size = I2C_MEMADD_SIZE_8BIT;
        return 1U;
    }

    if (register_address_size == BSP_ST_I2C_REGISTER_ADDRESS_16BIT)
    {
        *hal_register_address_size = I2C_MEMADD_SIZE_16BIT;
        return 1U;
    }

    return 0U;
}

HAL_StatusTypeDef BSP_ST_I2C_Init(void)
{
    uint32_t i;
    uint32_t j;
    uint32_t configured_count = 0U;

    for (i = 0U; i < BSP_ST_I2C_COUNT; i++)
    {
        if (s_i2c_handles[i] == NULL)
        {
            continue;
        }

        if ((s_i2c_handles[i]->Instance == NULL) ||
            (HAL_I2C_GetState(s_i2c_handles[i]) != HAL_I2C_STATE_READY))
        {
            return HAL_ERROR;
        }

        configured_count++;
        for (j = i + 1U; j < BSP_ST_I2C_COUNT; j++)
        {
            if (s_i2c_handles[i] == s_i2c_handles[j])
            {
                return HAL_ERROR;
            }
        }
    }

    if (configured_count == 0U)
    {
        return HAL_ERROR;
    }

    s_i2c_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef BSP_ST_I2C_IsDeviceReady(
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    uint32_t trials)
{
    I2C_HandleTypeDef *handle;

    if ((trials == 0U) ||
        (BSP_ST_I2C_GetHandle(i2c_id, device_address, &handle) != HAL_OK))
    {
        return HAL_ERROR;
    }

    return HAL_I2C_IsDeviceReady(
        handle,
        (uint16_t)((uint16_t)device_address << 1U),
        trials,
        BSP_ST_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef BSP_ST_I2C_Write(
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    const uint8_t *data,
    uint16_t length)
{
    I2C_HandleTypeDef *handle;

    if (((data == NULL) && (length != 0U)) ||
        (BSP_ST_I2C_GetHandle(i2c_id, device_address, &handle) != HAL_OK))
    {
        return HAL_ERROR;
    }

    if (length == 0U)
    {
        return HAL_OK;
    }

    return HAL_I2C_Master_Transmit(
        handle,
        (uint16_t)((uint16_t)device_address << 1U),
        (uint8_t *)data,
        length,
        BSP_ST_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef BSP_ST_I2C_Read(
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    uint8_t *data,
    uint16_t length)
{
    I2C_HandleTypeDef *handle;

    if (((data == NULL) && (length != 0U)) ||
        (BSP_ST_I2C_GetHandle(i2c_id, device_address, &handle) != HAL_OK))
    {
        return HAL_ERROR;
    }

    if (length == 0U)
    {
        return HAL_OK;
    }

    return HAL_I2C_Master_Receive(
        handle,
        (uint16_t)((uint16_t)device_address << 1U),
        data,
        length,
        BSP_ST_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef BSP_ST_I2C_WriteRegister(
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    uint16_t register_address,
    BSP_ST_I2C_RegisterAddressSize register_address_size,
    const uint8_t *data,
    uint16_t length)
{
    I2C_HandleTypeDef *handle;
    uint16_t hal_register_address_size;

    if (((data == NULL) && (length != 0U)) ||
        !BSP_ST_I2C_GetHalRegisterAddressSize(
            register_address_size,
            register_address,
            &hal_register_address_size) ||
        (BSP_ST_I2C_GetHandle(i2c_id, device_address, &handle) != HAL_OK))
    {
        return HAL_ERROR;
    }

    if (length == 0U)
    {
        return HAL_OK;
    }

    return HAL_I2C_Mem_Write(
        handle,
        (uint16_t)((uint16_t)device_address << 1U),
        register_address,
        hal_register_address_size,
        (uint8_t *)data,
        length,
        BSP_ST_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef BSP_ST_I2C_ReadRegister(
    BSP_ST_I2C_Id i2c_id,
    uint8_t device_address,
    uint16_t register_address,
    BSP_ST_I2C_RegisterAddressSize register_address_size,
    uint8_t *data,
    uint16_t length)
{
    I2C_HandleTypeDef *handle;
    uint16_t hal_register_address_size;

    if (((data == NULL) && (length != 0U)) ||
        !BSP_ST_I2C_GetHalRegisterAddressSize(
            register_address_size,
            register_address,
            &hal_register_address_size) ||
        (BSP_ST_I2C_GetHandle(i2c_id, device_address, &handle) != HAL_OK))
    {
        return HAL_ERROR;
    }

    if (length == 0U)
    {
        return HAL_OK;
    }

    return HAL_I2C_Mem_Read(
        handle,
        (uint16_t)((uint16_t)device_address << 1U),
        register_address,
        hal_register_address_size,
        data,
        length,
        BSP_ST_I2C_TIMEOUT_MS);
}
