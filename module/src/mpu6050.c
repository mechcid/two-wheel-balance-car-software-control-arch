#include "mpu6050.h"
#include <math.h>

/* HAL 接口要求将 7 位设备地址左移一位；所有传输均设置超时。 */
static HAL_StatusTypeDef read_reg(MPU6050 *d, uint8_t reg, uint8_t *data, uint16_t n)
{
    d->status = HAL_I2C_Mem_Read(d->bus, d->address << 1, reg,
                               I2C_MEMADD_SIZE_8BIT, data, n, 20);
    d->error = HAL_I2C_GetError(d->bus);
    return d->status;
}
static HAL_StatusTypeDef write_reg(MPU6050 *d, uint8_t reg, uint8_t value)
{
    d->status = HAL_I2C_Mem_Write(d->bus, d->address << 1, reg,
                                I2C_MEMADD_SIZE_8BIT, &value, 1, 20);
    d->error = HAL_I2C_GetError(d->bus);
    return d->status;
}
HAL_StatusTypeDef MPU6050_Init(MPU6050 *d, I2C_HandleTypeDef *bus)
{
    if (!d || !bus) return HAL_ERROR;
    *d = (MPU6050){.bus = bus};
    for (d->address = 0x68; d->address <= 0x69; ++d->address) {
        if (read_reg(d, 0x75, &d->who_am_i, 1) == HAL_OK && d->who_am_i == 0x68)
            break;
    }
    if (d->address > 0x69) return d->status = HAL_ERROR;
    if (write_reg(d, 0x6b, 0x80) != HAL_OK) return d->status;
    HAL_Delay(100);
    /* 以 X 轴陀螺仪作为 PLL 时钟参考，启用全部轴，数字低通滤波配置为 3，
     * 输出速率为 200 Hz，角速度量程为 ±250 度/秒，加速度量程为 ±2 g。 */
    const uint8_t config[][2] = {{0x6b,1},{0x6c,0},{0x1a,3},{0x19,4},{0x1b,0},{0x1c,0},{0x37,0},{0x38,1}};
    for (unsigned i = 0; i < sizeof(config)/sizeof(config[0]); ++i) {
        uint8_t actual = 0;
        if (write_reg(d, config[i][0], config[i][1]) != HAL_OK) return d->status;
        if (read_reg(d, config[i][0], &actual, 1) != HAL_OK) return d->status;
        if (actual != config[i][1]) return d->status = HAL_ERROR;
    }
    HAL_Delay(100);
    d->ready = 1;
    return HAL_OK;
}
static int16_t signed_be(const uint8_t *p)
{
    int32_t v = ((uint16_t)p[0] << 8) | p[1];
    return (int16_t)(v >= 32768 ? v - 65536 : v);
}
HAL_StatusTypeDef MPU6050_Read(MPU6050 *d, MPU6050_Sample *out)
{
    if (!d || !out || !d->ready) return HAL_ERROR;
    /* 本板使用 HAL 连续读取时，最后一个字节会重复倒数第二个字节；
       已通过连续读取 14/15 字节及单独读取 0x47/0x48 寄存器交叉确认。
       因此额外读取地址 0x49 的 EXT_SENS_DATA_00 字节，读取它没有副作用，
       使所需的 14 字节传感器数据全部位于异常尾字节之前。
       保留 MPU6050_Diagnose 函数用于复查。
       这是本板实测采用的规避措施，不代表所有 STM32F1 都存在此问题。 */
    uint8_t raw[15];
    if (read_reg(d, 0x3b, raw, sizeof(raw)) != HAL_OK) return d->status;
    /* 仅在完整读取成功后更新输出样本，避免将旧数据标记为新数据。 */
    *out = (MPU6050_Sample){signed_be(raw), signed_be(raw+2), signed_be(raw+4),
        signed_be(raw+8), signed_be(raw+10), signed_be(raw+12)};
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Diagnose(MPU6050 *d, MPU6050_Diagnostic *r)
{
    if (!d || !r || !d->ready) return HAL_ERROR;
    r->ok = 0;
    if (read_reg(d, 0x3b, r->burst14, 14) != HAL_OK ||
        read_reg(d, 0x3b, r->burst15, 15) != HAL_OK ||
        read_reg(d, 0x47, &r->separate[0], 1) != HAL_OK ||
        read_reg(d, 0x48, &r->separate[1], 1) != HAL_OK) return d->status;
    r->ok = 1;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_SetOutputRate(MPU6050 *d, float hz, float *actual)
{
    if (!d || !d->ready || !actual || !isfinite(hz) || hz<=0) return HAL_ERROR;
    float divider=1000.0f/hz;
    if (divider>256) divider=256;
    if (divider<1) divider=1;
    uint16_t n=(uint16_t)(divider+0.5f);
    uint8_t value=(uint8_t)(n-1), readback;
    if (write_reg(d,0x19,value)!=HAL_OK || read_reg(d,0x19,&readback,1)!=HAL_OK)
        return d->status;
    if (readback!=value) return d->status=HAL_ERROR;
    *actual=1000.0f/n;
    return HAL_OK;
}
HAL_StatusTypeDef MPU6050_DataReady(MPU6050 *d)
{
    if (!d || !d->ready) return HAL_ERROR;
    uint8_t value;
    if (read_reg(d,0x3a,&value,1)!=HAL_OK) return d->status;
    return (value & 1) ? HAL_OK : HAL_BUSY;
}
