#include "sensor_app.h"
#include "mpu6050.h"
#include "timebase.h"
#include <math.h>

static MPU6050 device;
static I2C_HandleTypeDef *sensor_bus;
static MPU6050_Sample raw_sample; /* 私有原始样本，不向外部使用者暴露。 */
static ImuSample corrected_sample;
static SensorStatus status;
static SensorCalibration calibration;
static float actual_rate_hz;

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
            !isfinite(value->accel_scale[i]) || fabsf(value->accel_scale[i]) > 10 || value->accel_scale[i] == 0 ||
            !isfinite(value->gyro_scale[i]) || fabsf(value->gyro_scale[i]) > 10 || value->gyro_scale[i] == 0)
            return false;
    }
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
    corrected_sample.valid=false;
    actual_rate_hz=0;
    HAL_StatusTypeDef result = MPU6050_Init(&device, sensor_bus);
    if (result==HAL_OK) result=MPU6050_SetOutputRate(&device,IMU_OUTPUT_HZ,&actual_rate_hz);
    if (result!=HAL_OK) device.ready=0;
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
    corrected_sample = (ImuSample){0};
    status = (SensorStatus){0};
    return SensorApp_Reconnect();
}
/* 换算系数对应底层驱动配置的 ±2 g、±250 度/秒量程。
 * 物理量校准统一在此完成，显示和遥测模块只使用修正结果。 */
static ImuSample correct(const MPU6050_Sample *raw, uint32_t timestamp_us){
    //默认的储存单位为 g、degree/s
    const int16_t accel[] = {raw->ax, raw->ay, raw->az};
    const int16_t gyro[] = {raw->gx, raw->gy, raw->gz};
    float a[3], g[3];
    for (unsigned i = 0; i < 3; ++i) {
        a[i] = ((float)accel[i] / 16384.0f - calibration.accel_bias_g[i]) * calibration.accel_scale[i];
        g[i] = ((float)gyro[i] / 131.0f - calibration.gyro_bias_dps[i]) * calibration.gyro_scale[i];
    }
    return (ImuSample){
        .data={.accel={a[0],a[1],a[2]}, .gyro={g[0],g[1],g[2]}},
        .timestamp_us=timestamp_us, .sequence=corrected_sample.sequence+1U, .valid=true};
}
HAL_StatusTypeDef SensorApp_Update(void)
{
    uint32_t now=Timebase_NowUs();
    if ((uint32_t)(now-corrected_sample.timestamp_us)>=IMU_TIMEOUT_US)
        corrected_sample.valid=false;
    if (!device.ready) return HAL_ERROR;
    MPU6050_Sample next_raw;
    HAL_StatusTypeDef result = MPU6050_DataReady(&device);
    if (result==HAL_BUSY) return HAL_BUSY;
    uint32_t timestamp_us=Timebase_NowUs();
    if (result==HAL_OK) result=MPU6050_Read(&device, &next_raw);
    if (result != HAL_OK) {
        corrected_sample.valid=false;
        status.failures++;
        device.ready = 0;
        capture_status(result);
        return result;
    }
    ImuSample next_corrected = correct(&next_raw, timestamp_us);
    raw_sample = next_raw;
    corrected_sample = next_corrected;
    status.reads++;
    capture_status(HAL_OK);
    return HAL_OK;
}

//定义两个返回只读指针的函数，用于获取状态信息
const ImuSample *SensorApp_GetSample(void) { return &corrected_sample; }
const SensorStatus *SensorApp_GetStatus(void) { return &status; }

float SensorApp_GetOutputRate(void) { return actual_rate_hz; }
