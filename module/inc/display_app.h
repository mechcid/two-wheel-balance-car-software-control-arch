#ifndef DISPLAY_APP_H
#define DISPLAY_APP_H
#include "sensor_app.h"
HAL_StatusTypeDef DisplayApp_Init(I2C_HandleTypeDef *bus);
void DisplayApp_Render(const SensorSample *sample, bool available);
void DisplayApp_Poll(void);
#endif
