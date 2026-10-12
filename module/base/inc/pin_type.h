#ifndef PIN_TYPE_H
#define PIN_TYPE_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 只描述已配置的硬件资源；不配置时钟、引脚复用，也不启动外设。 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin; /* 单个 GPIO_PIN_x 位掩码。 */
} GPIO_pin_t;

typedef struct {
    TIM_TypeDef *timer;
    uint8_t channel; /* 通道编号 1～4，不是 HAL 的 TIM_CHANNEL_x 编码。 */
} PWM_pin_t;

typedef struct {
    UART_HandleTypeDef *handle; /* 串口是外设资源，不是单个引脚。 */
} UART_pin_t;

typedef struct {
    GPIO_pin_t gpio;
    bool active_high; /* 用于使能、待机等具有有效电平语义的信号。 */
} GPIO_signal_t;

static inline bool GPIO_PinValid(GPIO_pin_t gpio)
{
    return gpio.port && gpio.pin && !(gpio.pin & (gpio.pin - 1u));
}
static inline bool GPIO_Write(GPIO_pin_t gpio, bool high)
{
    if (!GPIO_PinValid(gpio)) return false;
    HAL_GPIO_WritePin(gpio.port, gpio.pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}
static inline bool GPIO_SetActive(GPIO_signal_t signal, bool active)
{
    return GPIO_Write(signal.gpio, active == signal.active_high);
}

/* 约定：16 位定时器、向上计数、边沿对齐、PWM 模式 1、高电平有效。
 * ARR 必须在 1～65534 内，确保 CCR=ARR+1 可以表示完整的 100% 占空比。
 * 编码器定时器的 ARR=65535 不受此 PWM 限制影响。
 * 此函数只写比较寄存器；预装载是否启用决定何时真正生效。
 * 同一定时器的各 PWM 通道共享频率，本接口不修改 ARR 或预分频。 */
static inline bool PWM_PinValid(PWM_pin_t pwm)
{
    return pwm.timer && pwm.channel >= 1 && pwm.channel <= 4 &&
           pwm.timer->ARR >= 1 && pwm.timer->ARR <= 65534;
}
static inline bool PWM_SetDuty(PWM_pin_t pwm, float duty)
{
    if (!PWM_PinValid(pwm) || !(duty >= 0.0f && duty <= 1.0f)) return false;
    uint32_t compare = (uint32_t)((pwm.timer->ARR + 1u) * duty + 0.5f);
    switch (pwm.channel) {
    case 1: pwm.timer->CCR1 = compare; break;
    case 2: pwm.timer->CCR2 = compare; break;
    case 3: pwm.timer->CCR3 = compare; break;
    case 4: pwm.timer->CCR4 = compare; break;
    default: return false;
    }
    return true;
}

#ifdef __cplusplus
}
#endif
#endif
