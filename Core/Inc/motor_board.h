#ifndef MOTOR_BOARD_H
#define MOTOR_BOARD_H
#include "motor_app.h"
#ifdef __cplusplus
extern "C" {
#endif
/* 在 MX_GPIO_Init / MX_TIM1/3/4_Init 之后调用一次。
 * 检查配置并以零占空比启动计数/PWM，保持 STBY 关闭；失败返回 NULL。 */
const MotorAppConfig *MotorBoard_Init(void);
#ifdef __cplusplus
}
#endif
#endif
