#ifndef MOTOR_TEST_HAL_H
#define MOTOR_TEST_HAL_H
#include <stdint.h>
typedef struct { uint32_t ODR; } GPIO_TypeDef;
typedef struct { uint32_t ARR, CNT, CCR1, CCR2, CCR3, CCR4; } TIM_TypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState value)
{ if (value==GPIO_PIN_SET) port->ODR |= pin; else port->ODR &= ~(uint32_t)pin; }
#endif
