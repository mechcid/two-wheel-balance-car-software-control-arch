#ifndef MOTOR_APP_H
#define MOTOR_APP_H
#include "pin_type.h"
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* C 接口隔离 C++ 类；main 和串口模块无需改为 C++。 */
typedef enum { MOTOR_STOPPED, MOTOR_DUTY, MOTOR_SPEED, MOTOR_FAULT, MOTOR_UNCONFIGURED } MotorAppMode;
typedef struct {
    GPIO_pin_t direction1, direction2;
    PWM_pin_t pwm;
    TIM_TypeDef *encoder_timer;
    float counts_per_rev;
    bool output_reversed, encoder_reversed;
    float kp, ki, kd;
    bool incremental_pid;
} MotorWheelConfig;
typedef struct {
    MotorWheelConfig wheels[2]; /* 顺序：左、右。 */
    float duty_limit, rpm_limit;
    bool forward_in1, forward_in2, reverse_in1, reverse_in2, idle_in1, idle_in2;
    void (*set_driver_enabled)(bool enabled); /* 板级 STBY 回调；初始化和故障时关闭。 */
    bool speed_control_allowed; /* 参数及反馈极性确认后才能开启闭环。 */
} MotorAppConfig;
typedef struct {
    MotorAppMode mode;
    bool feedback_valid[2];
    float rpm[2], duty[2], target[2];
    uint32_t timestamp_ms, faults;
} MotorAppStatus;

bool MotorApp_Init(const MotorAppConfig *config, uint32_t now_ms);
void MotorApp_Update(uint32_t now_ms);
bool MotorApp_SetDuty(float left, float right, uint32_t now_ms);
bool MotorApp_SetRPM(float left, float right, uint32_t now_ms);
void MotorApp_Stop(uint32_t now_ms);
void MotorApp_KeepAlive(uint32_t now_ms);
bool MotorApp_IsActive(void);
const MotorAppStatus *MotorApp_GetStatus(void);
#ifdef __cplusplus
}
#endif
#endif
