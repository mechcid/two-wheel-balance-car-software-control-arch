#ifndef TELEMETRY_APP_H
#define TELEMETRY_APP_H
#include "sensor_app.h"
bool TelemetryApp_Init(UART_HandleTypeDef *uart);
void TelemetryApp_Poll(void);
void TelemetryApp_Report(const ImuSample *sample, bool available);
#endif
