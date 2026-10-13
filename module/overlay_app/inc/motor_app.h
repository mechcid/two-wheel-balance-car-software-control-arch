#ifndef MOTOR_APP_H
#define MOTOR_APP_H
#include "motor_types.h"
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

/* 单轮配置直接复用 MotorConfig，应用层只组合左右轮和共用 STBY。 */
typedef struct {
    MotorConfig left;
    MotorConfig right;
    void (*set_driver_enabled)(bool enabled);
} MotorAppConfig;
typedef struct {
    MotorAppMode mode;
    MotorState left;
    MotorState right;
    uint32_t timestamp_us, faults;
} MotorAppStatus;

/* 所有时间参数来自 Timebase_NowUs；主循环约每 10 ms 更新一次。
 * SetDuty 参数与状态 applied_duty 均为 -1..1 占空比，rpm 仅为实测反馈。 */
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
