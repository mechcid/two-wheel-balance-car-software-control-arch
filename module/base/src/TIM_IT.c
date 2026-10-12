#include "TIM_IT.h"

static TIM_HandleTypeDef *source;
static volatile uint32_t produced;
static uint32_t consumed;

bool TIM_IT_GetTiming(TIM_HandleTypeDef *timer, TIM_IT_Timing *out)
{
    if (!timer || !out || timer->Instance!=TIM2 ||
        (timer->Instance->CR1 & (TIM_CR1_DIR|TIM_CR1_CMS)) ||
        (timer->Instance->SMCR & (TIM_SMCR_SMS|TIM_SMCR_ECE))) return false;
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks,&latency);
    uint32_t hz=HAL_RCC_GetPCLK1Freq();
    if (clocks.APB1CLKDivider!=RCC_HCLK_DIV1) hz*=2;
    if (!hz) return false;
    out->clock_hz=hz;
    out->psc=timer->Instance->PSC;
    out->arr=timer->Instance->ARR;
    out->period_s=((float)out->psc+1)*((float)out->arr+1)/(float)hz;
    out->frequency_hz=1/out->period_s;
    return true;
}
HAL_StatusTypeDef TIM_IT_Start(TIM_HandleTypeDef *timer)
{
    TIM_IT_Timing timing;
    if (source || !TIM_IT_GetTiming(timer,&timing)) return HAL_ERROR;
    /* 装载 PSC，清除初始化产生的更新标志，再启动真实周期中断。 */
    timer->Instance->EGR=TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(timer,TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(timer,0);
    produced=consumed=0;
    source=timer;
    HAL_StatusTypeDef result=HAL_TIM_Base_Start_IT(timer);
    if (result!=HAL_OK) source=NULL;
    return result;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    /* 中断仅记节拍，不读取 I2C、不打印、不执行浮点解算。 */
    if (timer==source) ++produced;
}
uint32_t TIM_IT_TakeTicks(void)
{
    uint32_t latest=produced;
    uint32_t ticks=latest-consumed;
    consumed=latest;
    return ticks;
}
