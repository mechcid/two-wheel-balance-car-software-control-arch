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
/* 单轮测量快照；计数器读数为硬件原值，delta_count 已应用方向配置。 */
typedef struct {
    uint16_t encoder_count, previous_encoder_count;
    int32_t delta_count;
    int64_t total_counts;
    float rpm, velocity_mps, delta_displacement_m;
    bool total_counts_valid, delta_valid, feedback_valid;
} MotorFeedback;
typedef struct {
    MotorFeedback left, right;
    uint32_t timestamp_us, sequence, interval_us;
    bool valid;
    /* 与上一成功样本之间存在采样缺失；失败时置位，首份恢复样本携带此标记。 */
    bool discontinuity;
} MotorAppSample;
enum {
    MOTOR_LEFT_FEEDBACK_FAILED=1u<<0,
    MOTOR_RIGHT_FEEDBACK_FAILED=1u<<1,
    MOTOR_LEFT_OUTPUT_FAILED=1u<<2,
    MOTOR_RIGHT_OUTPUT_FAILED=1u<<3,
    MOTOR_COMMAND_TIMEOUT=1u<<4,
    MOTOR_SAMPLE_INTERVAL_INVALID=1u<<5,
    MOTOR_CONFIG_INVALID=1u<<6,
    MOTOR_LEFT_INIT_FAILED=1u<<7,
    MOTOR_RIGHT_INIT_FAILED=1u<<8
};
typedef struct {
    MotorAppMode mode;
    uint32_t faults; /* 原因位掩码；故障保留至显式 Stop 恢复或下一次正常采样。 */
    float left_applied_duty, right_applied_duty;
} MotorAppStatus;

/* 时间由调用者传入，使用同一 uint32_t 微秒基准。
 * 每轮 Poll 检查保护；仅 sample_requested=true 时进行常规采样。
 * 调度层在匹配的新姿态到来时请求采样，不建立独立电机采样相位。 */
bool MotorApp_Init(const MotorAppConfig *config, uint32_t now_us);
void MotorApp_Poll(uint32_t now_us, bool sample_requested);
bool MotorApp_SetDuty(float left, float right, uint32_t now_us);
/* Closed-loop output: apply immediately without resampling encoders. Main loop only. */
bool MotorApp_ApplyDuty(float left, float right, uint32_t now_us);
void MotorApp_Stop(uint32_t now_us);
void MotorApp_KeepAlive(uint32_t now_us);
bool MotorApp_IsActive(void);
const MotorAppStatus *MotorApp_GetStatus(void);
const MotorAppSample *MotorApp_GetSample(void);
/* 主循环同步通知，包含停机补采样及失败通知；禁止回调重入更新/控制接口。
 * 成功才推进样本序号，失败保留数值与时间、置 valid=false 并通知。
 * 消费者先处理无效/不连续，再按成功序号去重；不可保留指针作为历史副本。
 * GetSample 仅保留最新样本，积分消费者必须接收每次通知。 */
typedef void (*MotorAppFeedbackCallback)(const MotorAppSample *sample);
void MotorApp_SetFeedbackCallback(MotorAppFeedbackCallback callback);
#ifdef __cplusplus
}
#endif
#endif
