#ifndef TELEMETRY_APP_H
#define TELEMETRY_APP_H
#include "sensor_app.h"
#include "motor_app.h"
#include "status_state_app.h"
bool TelemetryApp_Init(UART_HandleTypeDef *uart);
void TelemetryApp_Poll(void);
/* 独立周期输出；调用者决定组合与周期。@stream:0 暂停全部，非零恢复。
 * IMU/POSE 沿用原帧；MOTOR 为时间_us、模式、左右mrpm、左右mm、
 * 左右测速有效性、左右位移有效性。手动查询命令不受 stream 控制。 */
void TelemetryApp_ReportImu(const ImuSample *sample, bool available);
void TelemetryApp_ReportAttitude(bool available);
void TelemetryApp_ReportMotor(const MotorAppStatus *status);
/* STATE：时间_us、序号、valid、faults、x_mm、y_mm、theta_mrad、psi_mrad、
 * v_mmps、theta_dot_mradps、psi_dot_mradps；无效帧也输出供诊断。 */
void TelemetryApp_ReportState(const StatusState *state);
#endif
