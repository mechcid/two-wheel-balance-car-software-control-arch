#ifndef DISPLAY_APP_H
#define DISPLAY_APP_H
#include "stm32f1xx_hal.h"
#include "motion_estimator.h"

HAL_StatusTypeDef DisplayApp_Init(I2C_HandleTypeDef *bus);
/* 四行页面：标题及 roll、pitch、yaw 三个姿态角，单位为度。
 * 由调用者传入只读快照和可用状态，本模块不采集数据、不推进算法。 */
void DisplayApp_Render(const MotionState *motion, const float degrees[3], bool available);
void DisplayApp_Poll(void);
#endif
