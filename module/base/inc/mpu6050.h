#ifndef MPU6050_H
#define MPU6050_H
#include "main.h"

//储存一个MPU6050传感器的状态和配置，包括I2C总线、设备地址、WHO_AM_I寄存器值、初始化状态和错误码。
typedef struct {
    I2C_HandleTypeDef *bus;
    uint8_t address, who_am_i, ready;
    HAL_StatusTypeDef status;
    uint32_t error;
} MPU6050;

typedef struct {
    int16_t ax, ay, az, gx, gy, gz;
} MPU6050_Sample;


HAL_StatusTypeDef MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *bus);
HAL_StatusTypeDef MPU6050_Read(MPU6050 *dev, MPU6050_Sample *sample);
/* DLPF=3 时基础输出为 1 kHz；请求频率取最近的合法整数分频。
 * 返回实际频率；仅在主循环调用。 */
HAL_StatusTypeDef MPU6050_SetOutputRate(MPU6050 *dev, float requested_hz, float *actual_hz);
/* 无新样本返回 HAL_BUSY，不表示总线故障。 */
HAL_StatusTypeDef MPU6050_DataReady(MPU6050 *dev);

/* 交叉诊断使用连续的多次读取事务，仅适合在设备静止时比较结果。 */
typedef struct { 
    uint8_t burst14[14], burst15[15], separate[2]; 
    uint8_t ok; 
} MPU6050_Diagnostic;

HAL_StatusTypeDef MPU6050_Diagnose(MPU6050 *dev, MPU6050_Diagnostic *result);
#endif
