#ifndef ENCODERS_H
#define ENCODERS_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 每个编码器拥有自己的状态，不再使用全局数组和编号。
 * timer 指向已配置并启动的硬件编码器定时器，ARR 必须为 65535。
 * 每次更新间隔内的真实位移绝对值必须小于 32768 个计数。
 * 同一个实例只能由一个调用者更新；其他使用者只读取结果。 */
typedef struct {
    TIM_TypeDef *timer;
    uint16_t last_cnt; /* 本次计算所用的上次计数。 */
    uint16_t now_cnt;  /* 最近一次读取的计数。 */
    int16_t delta_cnt; /* 最近两次有效采样之间的有符号增量。 */
    bool valid;        // 基准是否有效；恰好半范围的差值可检测，其他超范围位移须靠采样周期避免。
} Encoder;

/* 初始化与重新建立基准均只读取计数器，不清零硬件计数。 */
bool Encoder_Init(Encoder *encoder);
bool Encoder_Update(Encoder *encoder);

#ifdef __cplusplus
}
#endif
#endif
