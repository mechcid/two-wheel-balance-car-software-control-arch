#ifndef TIMEBASE_H
#define TIMEBASE_H
#include <stdbool.h>
#include <stdint.h>
/* 固定整数 MHz 内核时钟；启动后不再复位 DWT。
 * 至少每个 CYCCNT 回绕周期调用一次（72 MHz 下 <59.65 s）。
 * 不覆盖休眠、长时间调试暂停或运行中变频。返回值约 71.6 分钟回绕；
 * 时间间隔以 uint32_t 无符号减法计算。可从主循环或普通中断调用。 */
bool Timebase_Init(void);
uint32_t Timebase_NowUs(void);
#endif
