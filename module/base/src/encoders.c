#include "encoders.h"

//检查编码器配置是否合法，存在编码器地址，TIM地址，并且ARR为65535，才算合法
static bool hardware_valid(const Encoder *encoder){
    return encoder && encoder->timer && encoder->timer->ARR == UINT16_MAX;
}
//重置编码器的last_cnt和now_cnt为当前计数器值，delta_cnt为0，valid为true
//只是用于刷新编码器计时状态，可以只在初始化时调用一次
bool Encoder_Init(Encoder *encoder){
    if (!hardware_valid(encoder)) {
        if (encoder) encoder->valid = false;
        return false;
    }
    uint16_t count = (uint16_t)encoder->timer->CNT;
    encoder->last_cnt = count;
    encoder->now_cnt = count;
    encoder->delta_cnt = 0;
    encoder->valid = true;
    return true;
}

bool Encoder_Update(Encoder *encoder){
    if (!hardware_valid(encoder) || !encoder->valid) {
        if(encoder) encoder->valid = false;
        return false;
    }
    uint16_t next = (uint16_t)encoder->timer->CNT;
    uint16_t difference = (uint16_t)(next - encoder->now_cnt);
    /* 恰好半圈计数无法判断正反方向，要求调用者重新建立基准。
     * 超过半圈的其他位移无法仅凭前后两个读数识别，必须靠采样周期保证。 */
    if (difference == 32768u) {
        encoder->valid = false;
        return false;
    }
    int16_t signed_difference = (int16_t)difference;
    encoder->last_cnt = encoder->now_cnt;
    encoder->now_cnt = next;
    encoder->delta_cnt = signed_difference;
    return true;
}
