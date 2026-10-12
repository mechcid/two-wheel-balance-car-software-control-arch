#ifndef MOTOR_APP_H
#define MOTOR_APP_H
#include "pin_type.h"
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* C 接口隔离 C++ 类；main 和串口模块无需改为 C++。 */

//枚举定义了运动组织中的四种行为，电机停止，运行，出错，未识别到电机类型
typedef enum { 
    MOTOR_STOPPED, 
    MOTOR_DUTY,
    MOTOR_FAULT, 
    MOTOR_UNCONFIGURED 
} MotorAppMode;

//定义要传给电机的配置，和motor.hpp里面的MotorConfig差不多
typedef struct {
    GPIO_pin_t direction1, direction2;
    PWM_pin_t pwm;
    TIM_TypeDef *encoder_timer;
    float counts_per_rev;
    bool output_reversed, encoder_reversed;
} MotorWheelConfig;


typedef struct {
    MotorWheelConfig left;
    MotorWheelConfig right;
    float duty_limit;
    /* IN1/IN2 电平组合：true=高，false=低；由驱动器真值表决定。
     * forward/reverse 用于正反向输出；idle 用于单轮 stop，另将 PWM 置零。
     * 总停机还会通过下方回调关闭 STBY。 */
    bool forward_in1, forward_in2, reverse_in1, reverse_in2, idle_in1, idle_in2;
    void (*set_driver_enabled)(bool enabled); /* 板级 STBY 回调；初始化和故障时关闭。 */
} MotorAppConfig;
typedef struct {
    float rpm;
    float duty;
    bool feedback_valid;
} MotorWheelStatus;
typedef struct {
    MotorAppMode mode;
    MotorWheelStatus left;
    MotorWheelStatus right;
    uint32_t timestamp_us, faults;
} MotorAppStatus;

/* 所有时间参数来自 Timebase_NowUs；主循环约每 10 ms 更新一次。
 * SetDuty 参数与状态 duty 均为 -1..1 占空比，rpm 仅为实测反馈。 */
bool MotorApp_Init(const MotorAppConfig *config, uint32_t now_us);
void MotorApp_Update(uint32_t now_us);
bool MotorApp_SetDuty(float left, float right, uint32_t now_us);
void MotorApp_Stop(uint32_t now_us);
void MotorApp_KeepAlive(uint32_t now_us);
bool MotorApp_IsActive(void);
const MotorAppStatus *MotorApp_GetStatus(void);
#ifdef __cplusplus
}
#endif
#endif
