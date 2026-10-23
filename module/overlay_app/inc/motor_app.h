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
    uint32_t sequence;    /* 每次双轮采样发布递增（含无效帧），自然回绕。初始化不发布。 */
    uint32_t interval_us; /* 本次与上次采样的间隔；timestamp_us 是区间末端。 */
    bool valid;          /* 两轮增量同区间有效；不表示未被消费者处理。 */
} MotorAppStatus;

/* 所有时间参数来自 Timebase_NowUs；主循环每 5 ms 更新一次。
 * SetDuty 参数与状态 applied_duty 均为 -1..1 占空比，rpm 仅为实测反馈。 */
bool MotorApp_Init(const MotorAppConfig *config, uint32_t now_us);
void MotorApp_Update(uint32_t now_us);
bool MotorApp_SetDuty(float left, float right, uint32_t now_us);
/* Closed-loop output: apply immediately without resampling encoders. Main loop only. */
bool MotorApp_ApplyDuty(float left, float right, uint32_t now_us);
void MotorApp_Stop(uint32_t now_us);
void MotorApp_KeepAlive(uint32_t now_us);
bool MotorApp_IsActive(void);
const MotorAppStatus *MotorApp_GetStatus(void);
/* 主循环同步通知，包括启停补采样和无效帧；初始化前也可注册，NULL 取消。
 * 回调必须短小，不得调用 MotorApp 的更新/控制接口或保留指针作历史数据。
 * 里程计在此消费每一帧；GetStatus 仅保留最新帧，低频轮询会漏帧。
 * 消费者各自保存 sequence，重复帧跳过，跳号/valid=false 标记连续性丢失。
 * 读取不会清除 valid；控制状态变化本身不产生新采样序号。 */
typedef void (*MotorAppFeedbackCallback)(const MotorAppStatus *sample);
void MotorApp_SetFeedbackCallback(MotorAppFeedbackCallback callback);
#ifdef __cplusplus
}
#endif
#endif
