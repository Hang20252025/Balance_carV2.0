# `app_st_imu` 使用说明

`app_st_imu` 提供基于 MPU6050 的姿态角读取接口，内部通过 I2C 读取加速度计和陀螺仪数据，并使用互补滤波计算俯仰角和横滚角。

接口声明位于 [`app_st_imu.h`](./app_st_imu.h)，实现位于 [`app_st_imu.c`](./app_st_imu.c)。

## 依赖

使用前需要准备：

1. 在 CubeMX 中配置 I2C 外设，并生成对应的初始化代码和 I2C 句柄。
2. 在 [`bsp_st_i2c.h`](../bsp/bsp_st_i2c.h) 中将总线映射到 CubeMX 句柄：
   - 本模块固定使用 `BSP_ST_I2C_ID_2`
   - 因此 `BSP_ST_I2C_2_HANDLE` 应指向实际连接 MPU6050 的句柄，例如 `&hi2c2`
3. 按 [`bsp_st_i2c` 使用说明](../bsp/bsp_st_i2c.md) 调用 I2C 底层初始化。
4. 确保 MPU6050 的设备地址为 `0x68`。若模块使用地址引脚改变地址，需要同步修改 `APP_ST_IMU_ADDRESS`。

## 数据结构

```c
typedef struct
{
    float roll_deg;
    float pitch_deg;
    float gyro_x_deg_s;
    float gyro_y_deg_s;
    float gyro_z_deg_s;
} APP_ST_IMU_Data;
```

| 字段             | 含义                         | 单位  |
| ---------------- | ---------------------------- | ----- |
| `roll_deg`     | 横滚角                       | 度    |
| `pitch_deg`    | 俯仰角                       | 度    |
| `gyro_x_deg_s` | X 轴角速度，已减去初始化零偏 | 度/秒 |
| `gyro_y_deg_s` | Y 轴角速度，已减去初始化零偏 | 度/秒 |
| `gyro_z_deg_s` | Z 轴角速度，已减去初始化零偏 | 度/秒 |

加速度计和陀螺仪使用 MPU6050 的默认量程：

- 加速度计量程：`±2 g`
- 陀螺仪量程：`±250 °/s`

## 初始化

```c
HAL_StatusTypeDef APP_ST_IMU_Init(uint32_t calibration_samples);
```

初始化流程：

1. 在总线 `BSP_ST_I2C_ID_2` 上读取 MPU6050 的 `WHO_AM_I` 寄存器。
2. 验证设备 ID 是否为 MPU6050 的 `0x68`。
3. 配置采样分频、数字低通滤波器和加速度计/陀螺仪量程。
4. 每隔 5 ms 读取一次陀螺仪原始值并计算零偏。
5. 将姿态角和角速度初始化为零。

参数 `calibration_samples` 是陀螺仪零偏校准的采样次数：

- 传入大于 `0` 的值时使用指定次数。
- 传入 `0` 时使用默认的 400 次。

校准期间设备应保持静止。

示例：

```c
HAL_StatusTypeDef status;

status = BSP_ST_I2C_Init();
if (status != HAL_OK)
{
    Error_Handler();
}

status = APP_ST_IMU_Init(400U);
if (status != HAL_OK)
{
    Error_Handler();
}
```

`APP_ST_IMU_Init()` 返回：

- `HAL_OK`：设备识别成功、配置成功并完成校准。
- `HAL_ERROR`：I2C 未就绪、设备未应答、设备 ID 不符、寄存器配置失败或读取失败。

## 周期更新

```c
HAL_StatusTypeDef APP_ST_IMU_Update(void);
```

每次更新会读取一次 MPU6050 原始数据，根据实际 `HAL_GetTick()` 间隔计算姿态角，并更新角速度数据。

建议按约 5 ms 周期调用一次：

```c
if (APP_ST_IMU_Update() != HAL_OK)
{
    /* 当前周期读取失败，保留上次姿态数据 */
}
```

调用后应使用下一节中的接口读取最新数据。若定时周期异常（小于等于 0.1 ms 或大于 100 ms），内部会按 5 ms 计算一次。

## 获取数据

一次性复制全部姿态数据：

```c
APP_ST_IMU_Data data;

APP_ST_IMU_GetData(&data);
```

获取俯仰角：

```c
float pitch_deg = APP_ST_IMU_GetPitchAngle();
```

获取横滚角：

```c
float roll_deg = APP_ST_IMU_GetRollAngle();
```

接口原型：

```c
void APP_ST_IMU_GetData(APP_ST_IMU_Data *data);
float APP_ST_IMU_GetPitchAngle(void);
float APP_ST_IMU_GetRollAngle(void);
```

传入空指针时，`APP_ST_IMU_GetData()` 不执行赋值。

## 主循环示例

```c
#include "app_st_imu.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C2_Init();

    if (BSP_ST_I2C_Init() != HAL_OK)
    {
        Error_Handler();
    }

    if (APP_ST_IMU_Init(400U) != HAL_OK)
    {
        Error_Handler();
    }

    while (1)
    {
        if (APP_ST_IMU_Update() == HAL_OK)
        {
            APP_ST_IMU_Data imu_data;
            APP_ST_IMU_GetData(&imu_data);

            /* 使用 imu_data.roll_deg、imu_data.pitch_deg 和陀螺仪数据 */
        }

        HAL_Delay(5U);
    }
}
```

实际项目中应使用主循环定时、定时器中断或实时任务按固定周期调用 `APP_ST_IMU_Update()`，避免在未初始化或 I2C 失败时继续读取数据。

## 姿态计算说明

- 横滚角由加速度计的 `Y/Z` 分量计算。
- 俯仰角由加速度计的 `X` 分量和 `Y/Z` 合成值计算。
- 姿态角采用互补滤波，陀螺仪占 `98%`，加速度计占 `2%`。
- MPU6050 设备内部应保持静止以完成零偏校准。
- 模块依赖外部 I2C 和 HAL 时基；硬件上电配置、I2C 引脚、时钟和总线句柄均由 CubeMX 管理。
