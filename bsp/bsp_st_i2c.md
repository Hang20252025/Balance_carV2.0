# `bsp_st_i2c` 使用说明

`bsp_st_i2c` 通过总线编号提供通用 I2C 主机读写接口。应用层只需传入总线编号和从设备地址，无需直接使用 CubeMX 的 I2C 句柄。接口实现位于 [`bsp_st_i2c.c`](./bsp_st_i2c.c)，总线映射和接口声明位于 [`bsp_st_i2c.h`](./bsp_st_i2c.h)。

## 配置与初始化

| 步骤 | 操作 |
|---|---|
| 1 | 在 CubeMX 配置 I2C 外设并生成初始化代码 |
| 2 | 在 `bsp_st_i2c.h` 中将 `BSP_ST_I2C_1_HANDLE` 或 `BSP_ST_I2C_2_HANDLE` 映射到对应的 `&hi2cX`；未使用的总线保留 `NULL` |
| 3 | 若映射使用 `hi2cX`，在 `bsp_st_i2c.c` 中先包含生成的 `i2c.h`，再包含 `bsp_st_i2c.h` |
| 4 | 先调用对应的 `MX_I2CX_Init()`，再调用 `BSP_ST_I2C_Init()` |

`BSP_ST_I2C_Init()` 会检查已映射句柄是否已由 HAL 初始化、是否重复映射；没有配置任何总线或句柄未就绪时返回 `HAL_ERROR`。I2C 配置参数、引脚和时钟仍由 CubeMX 管理。

## 总线编号和地址

| 总线编号 | 映射宏 |
|---|---|
| `BSP_ST_I2C_ID_1` | `BSP_ST_I2C_1_HANDLE` |
| `BSP_ST_I2C_ID_2` | `BSP_ST_I2C_2_HANDLE` |

所有接口的 `device_address` 都使用未左移的 7 位地址，例如设备地址 `0x68` 直接传 `0x68`。接口内部会按 STM32 HAL 的要求转换地址。

## 常用接口

| 接口 | 用途 |
|---|---|
| `BSP_ST_I2C_IsDeviceReady()` | 检查设备是否应答 |
| `BSP_ST_I2C_Write()` / `BSP_ST_I2C_Read()` | 直接发送或接收数据 |
| `BSP_ST_I2C_WriteRegister()` / `BSP_ST_I2C_ReadRegister()` | 访问带 8 位或 16 位寄存器地址的设备 |

读写接口为阻塞调用，超时时间由 `BSP_ST_I2C_TIMEOUT_MS` 控制（默认 100 ms）。寄存器地址宽度通过 `BSP_ST_I2C_REGISTER_ADDRESS_8BIT` 或 `BSP_ST_I2C_REGISTER_ADDRESS_16BIT` 指定；使用 8 位宽度时，寄存器地址须不大于 `0xFF`。

```c
uint8_t value;

if (BSP_ST_I2C_ReadRegister(
        BSP_ST_I2C_ID_1,
        0x68U,
        0x00U,
        BSP_ST_I2C_REGISTER_ADDRESS_8BIT,
        &value,
        1U) != HAL_OK)
{
    Error_Handler();
}
```

接口返回 HAL 状态：`HAL_OK` 表示操作成功；无效总线、地址、参数或未初始化时返回 `HAL_ERROR`。底层 HAL 也可能返回超时或忙碌状态。
