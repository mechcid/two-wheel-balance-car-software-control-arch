#include "timebase.h"
#include "stm32f1xx_hal.h"

static uint32_t last_cycles, elapsed_us, remainder, cycles_per_us;
static bool initialized;

bool Timebase_Init(void)
{
    if (initialized) return true;
    uint32_t hz=HAL_RCC_GetHCLKFreq();
    if (!hz || hz%1000000U) return false;
    cycles_per_us=hz/1000000U;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    last_cycles=elapsed_us=remainder=0;
    initialized=true;
    return true;
}
uint32_t Timebase_NowUs(void)
{
    /* 状态更新很短；保存 PRIMASK，避免嵌套调用破坏累计值。 */
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    if (!initialized) { __set_PRIMASK(mask); return 0; }
    uint32_t cycles=DWT->CYCCNT;
    uint32_t delta=cycles-last_cycles;
    last_cycles=cycles;
    /* 先除再累计余数，避免 delta + remainder 在周期回绕附近溢出。 */
    elapsed_us+=delta/cycles_per_us;
    remainder+=delta%cycles_per_us;
    elapsed_us+=remainder/cycles_per_us;
    remainder%=cycles_per_us;
    uint32_t result=elapsed_us;
    __set_PRIMASK(mask);
    return result;
}
