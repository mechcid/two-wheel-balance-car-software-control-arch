#include "sensor_app.h"
#include "mpu6050.h"
#include <math.h>

static MPU6050 device;
static I2C_HandleTypeDef *sensor_bus;
static MPU6050_Sample raw_sample; /* 私有原始样本，不向外部使用者暴露。 */
static SensorSample corrected_sample;
static SensorStatus status;
static SensorCalibration calibration;

//定义放缩倍数和零点偏移，主要在检查输入的值是否合法，合法了才会更新校准参数。起校准作用
//主要是给外部调用用的，在保持calibration为当前文件私有的同时提供一个外部更新的接口，用于需要动态校准的场景
bool SensorApp_SetCalibration(const SensorCalibration *value){
    //如果没有系数传入就恢复默认参数，零偏为 0、比例为 1
    if (!value) {
        calibration = (SensorCalibration){.accel_scale = {1,1,1}, .gyro_scale = {1,1,1}};
        return true;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(value->accel_bias_g[i]) || fabsf(value->accel_bias_g[i]) > 16 ||
            !isfinite(value->gyro_bias_dps[i]) || fabsf(value->gyro_bias_dps[i]) > 2000 ||
            !isfinite(value->accel_scale[i]) || value->accel_scale[i] > 10 ||
            !isfinite(value->gyro_scale[i]) || value->gyro_scale[i] > 10)
            return false;
    }
    if (!isfinite(value->temperature_offset_c) || fabsf(value->temperature_offset_c) > 200)
        return false;
    calibration = *value;
    return true;
}
static void capture_status(HAL_StatusTypeDef result){
    status.ready = device.ready != 0;
    status.address = device.address;
    status.who_am_i = device.who_am_i;
    status.last_result = result;
    status.bus_error = device.error;
}
HAL_StatusTypeDef SensorApp_Reconnect(void)
{
    HAL_StatusTypeDef result = MPU6050_Init(&device, sensor_bus);
    capture_status(result);
    return result;
}
HAL_StatusTypeDef SensorApp_Init(I2C_HandleTypeDef *bus, const SensorCalibration *initial_calibration)
{
    /* 先检查并复制配置，非法参数不清空已有样本，也不启动硬件初始化。 */
    if (!bus || !SensorApp_SetCalibration(initial_calibration)) return HAL_ERROR;
    sensor_bus = bus;
    device = (MPU6050){0};
    raw_sample = (MPU6050_Sample){0};
    corrected_sample = (SensorSample){0};
    status = (SensorStatus){0};
    return SensorApp_Reconnect();
}
/* 换算系数对应底层驱动配置的 ±2 g、±250 度/秒量程。
 * 物理量校准统一在此完成，显示和遥测模块只使用修正结果。 */
static SensorSample correct(const MPU6050_Sample *raw, uint32_t now_ms){
    //默认的储存单位为g，degree/s，温度为摄氏度
    const int16_t accel[] = {raw->ax, raw->ay, raw->az};
    const int16_t gyro[] = {raw->gx, raw->gy, raw->gz};
    float a[3], g[3];
    for (unsigned i = 0; i < 3; ++i) {
        a[i] = ((float)accel[i] / 16384.0f - calibration.accel_bias_g[i]) * calibration.accel_scale[i];
        g[i] = ((float)gyro[i] / 131.0f - calibration.gyro_bias_dps[i]) * calibration.gyro_scale[i];
    }
    return (SensorSample){
        .ax_g=a[0], .ay_g=a[1], .az_g=a[2],
        .gx_dps=g[0], .gy_dps=g[1], .gz_dps=g[2],
        .temperature_c=(float)raw->temperature / 340.0f + 36.53f + calibration.temperature_offset_c,
        .timestamp_ms=now_ms, .valid=true};
}
HAL_StatusTypeDef SensorApp_Update(uint32_t now_ms)
{
    if (!device.ready) return HAL_ERROR;
    MPU6050_Sample next_raw;
    HAL_StatusTypeDef result = MPU6050_Read(&device, &next_raw);
    if (result != HAL_OK) {
        status.failures++;
        device.ready = 0;
        capture_status(result);
        return result;
    }
    SensorSample next_corrected = correct(&next_raw, now_ms);
    raw_sample = next_raw;
    corrected_sample = next_corrected;
    status.reads++;
    capture_status(HAL_OK);
    return HAL_OK;
}

//定义两个返回只读指针的函数，用于获取状态信息
const SensorSample *SensorApp_GetSample(void) { return &corrected_sample; }
const SensorStatus *SensorApp_GetStatus(void) { return &status; }
