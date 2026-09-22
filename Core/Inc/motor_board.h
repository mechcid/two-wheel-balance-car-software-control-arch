#ifndef MOTOR_BOARD_H
#define MOTOR_BOARD_H
#include "motor_app.h"
#ifdef __cplusplus
extern "C" {
#endif
/* 板级映射和计数参数尚未确认时返回 NULL，不配置或驱动电机引脚。 */
const MotorAppConfig *MotorBoard_Init(void);
#ifdef __cplusplus
}
#endif
#endif
