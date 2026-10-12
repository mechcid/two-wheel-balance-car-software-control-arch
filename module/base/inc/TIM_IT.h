#ifndef TIM_IT_H
#define TIM_IT_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>

typedef struct {
    uint32_t clock_hz, psc, arr;
    float frequency_hz, period_s;
} TIM_IT_Timing;
/* 当前入口仅绑定 TIM2 的内部时钟、向上计数模式。 */
bool TIM_IT_GetTiming(TIM_HandleTypeDef *timer, TIM_IT_Timing *timing);
HAL_StatusTypeDef TIM_IT_Start(TIM_HandleTypeDef *timer);
/* 返回自上次取出以来发生的节拍数量；积压时只处理最新数据，不补读历史帧。 */
uint32_t TIM_IT_TakeTicks(void);
#endif
