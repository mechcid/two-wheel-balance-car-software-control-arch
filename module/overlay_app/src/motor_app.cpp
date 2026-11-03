#include "motor_app.h"
#include "motor.hpp"
#include <new>

namespace {
// 静态存储承载硬件所有者；初始化时原位构造，不使用堆分配。
alignas(Motor) unsigned char left_motor_storage[sizeof(Motor)];
alignas(Motor) unsigned char right_motor_storage[sizeof(Motor)];
Motor *left_motor{}, *right_motor{};
MotorAppConfig config{};
MotorAppSample sample{};
MotorAppStatus status{MOTOR_UNCONFIGURED, 0, 0, 0};
struct MotorCommand {
    float left_duty, right_duty;
    uint32_t timestamp_us;
};
MotorCommand command{};
MotorAppFeedbackCallback feedback_callback{};
uint32_t last_update;
bool initialized, pending_gap;
constexpr uint32_t command_timeout_us=1000000;
constexpr uint32_t max_control_gap_us=50000;

void sync_output() {
    status.left_applied_duty=left_motor->state().applied_duty;
    status.right_applied_duty=right_motor->state().applied_duty;
}
MotorFeedback feedback(const MotorState &s, const MotorConfig &c) {
    MotorFeedback f{};
    f.encoder_count=c.encoder->now_cnt;
    f.previous_encoder_count=c.encoder->last_cnt;
    f.delta_count=c.encoder_reversed ? -int32_t(c.encoder->delta_cnt) : c.encoder->delta_cnt;
    f.total_counts=s.total_counts;
    f.rpm=s.rpm; f.velocity_mps=s.velocity_mps;
    f.delta_displacement_m=s.delta_displacement_m;
    f.total_counts_valid=s.total_counts_valid;
    f.delta_valid=s.delta_valid; f.feedback_valid=s.feedback_valid;
    return f;
}
// 发布完整测量；失败也通知下游，但不把旧数值标记为新样本。
void publish(uint32_t now, uint32_t elapsed, bool feedback_ok) {
    const auto left=feedback(left_motor->state(),config.left);
    const auto right=feedback(right_motor->state(),config.right);
    bool valid=feedback_ok && elapsed>0 && elapsed<=max_control_gap_us &&
        left.delta_valid && right.delta_valid && left.feedback_valid && right.feedback_valid;
    if (valid) {
        sample={left,right,now,sample.sequence+1,elapsed,true,pending_gap};
        pending_gap=false;
    } else {
        sample.valid=false;
        sample.discontinuity=true;
        pending_gap=true;
    }
    if (feedback_callback) feedback_callback(&sample);
}
uint32_t stop_outputs() {
    config.set_driver_enabled(false);
    bool left=left_motor->stop(), right=right_motor->stop();
    sync_output();
    command={};
    return (left ? 0U : uint32_t(MOTOR_LEFT_OUTPUT_FAILED)) |
           (right ? 0U : uint32_t(MOTOR_RIGHT_OUTPUT_FAILED));
}
void stop_and_sample(uint32_t now, uint32_t reason) {
    uint32_t elapsed=now-last_update;
    float dt=(elapsed>0 && elapsed<=max_control_gap_us) ? elapsed*0.000001f : 0;
    uint32_t faults=reason | stop_outputs();
    bool left=left_motor->rebaseFeedback(dt);
    bool right=right_motor->rebaseFeedback(dt);
    if (!left) faults|=MOTOR_LEFT_FEEDBACK_FAILED;
    if (!right) faults|=MOTOR_RIGHT_FEEDBACK_FAILED;
    if (!elapsed || elapsed>max_control_gap_us) faults|=MOTOR_SAMPLE_INTERVAL_INVALID;
    status.faults=faults;
    // 零区间补采样不可用，但显式 Stop 仍可成功关闭输出并解除运行故障。
    status.mode=(reason || !left || !right ||
        (faults & (MOTOR_LEFT_OUTPUT_FAILED|MOTOR_RIGHT_OUTPUT_FAILED))) ? MOTOR_FAULT : MOTOR_STOPPED;
    last_update=now;
    publish(now,elapsed,left && right);
}
bool request_valid(float left,float right) {
    return initialized && status.mode!=MOTOR_FAULT &&
        std::isfinite(left) && std::isfinite(right) &&
        std::fabs(left)<=config.left.duty_limit && std::fabs(right)<=config.right.duty_limit;
}
uint32_t apply_output(float left,float right) {
    bool l=left_motor->setDuty(left), r=right_motor->setDuty(right);
    sync_output();
    return (l ? 0U : uint32_t(MOTOR_LEFT_OUTPUT_FAILED)) | (r ? 0U : uint32_t(MOTOR_RIGHT_OUTPUT_FAILED));
}
// 单次更新：采集双轮、执行待处理命令、发布测量，不管理触发周期。
void MotorApp_Update(uint32_t now) {
    uint32_t elapsed=now-last_update;
    last_update=now;
    bool left=left_motor->updateFeedback(elapsed*0.000001f);
    bool right=right_motor->updateFeedback(elapsed*0.000001f);
    uint32_t faults=0;
    if (!left) faults|=MOTOR_LEFT_FEEDBACK_FAILED;
    if (!right) faults|=MOTOR_RIGHT_FEEDBACK_FAILED;
    if (elapsed>max_control_gap_us) faults|=MOTOR_SAMPLE_INTERVAL_INVALID;
    if (!faults && status.mode==MOTOR_DUTY)
        faults|=apply_output(command.left_duty,command.right_duty);
    if (faults) {
        faults|=stop_outputs(); // 不重复采集、覆盖本次反馈。
        status.mode=MOTOR_FAULT;
    } else if (status.mode==MOTOR_DUTY) {
        config.set_driver_enabled(true);
    }
    status.faults=faults;
    publish(now,elapsed,left && right);
}
}

bool MotorApp_Init(const MotorAppConfig *settings, uint32_t now){
    if (initialized || left_motor) return false; // 仅启动时初始化一次，避免重复构造硬件所有者。
    //校验传入配置的合法性，主要是检查编码器和占空比限制
    if (!settings || !settings->set_driver_enabled ||
        !std::isfinite(settings->left.duty_limit) || settings->left.duty_limit<=0 || settings->left.duty_limit>1 ||
        !std::isfinite(settings->right.duty_limit) || settings->right.duty_limit<=0 || settings->right.duty_limit>1 ||
        !settings->left.encoder || !settings->right.encoder ||
        settings->left.encoder==settings->right.encoder ||
        !settings->left.encoder->timer || !settings->right.encoder->timer ||
        settings->left.encoder->timer==settings->right.encoder->timer)
        { status.faults=MOTOR_CONFIG_INVALID; return false; }
    config = *settings;
    config.set_driver_enabled(false);
    left_motor = new (left_motor_storage) Motor(config.left);
    if (!left_motor->begin()) {
        left_motor->stop();
        status.mode=MOTOR_FAULT;
        status.faults=MOTOR_LEFT_INIT_FAILED;
        return false;
    }
    right_motor = new (right_motor_storage) Motor(config.right);
    if (!right_motor->begin()) {
        left_motor->stop();
        right_motor->stop();
        status.mode=MOTOR_FAULT;
        status.faults=MOTOR_RIGHT_INIT_FAILED;
        return false;
    }
    initialized=true;
    last_update=command.timestamp_us=now;
    status.mode=MOTOR_STOPPED;
    status.faults=0;
    sync_output();
    return true;
}
void MotorApp_Poll(uint32_t now, bool sample_requested) {
    if (!initialized || status.mode==MOTOR_FAULT) return;
    uint32_t elapsed=now-last_update;
    uint32_t faults=0;
    if (MotorApp_IsActive()) {
        if (elapsed>max_control_gap_us) faults|=MOTOR_SAMPLE_INTERVAL_INVALID;
        if ((uint32_t)(now-command.timestamp_us)>=command_timeout_us) faults|=MOTOR_COMMAND_TIMEOUT;
    }
    if (faults) { stop_and_sample(now,faults); return; }
    if (sample_requested && elapsed) MotorApp_Update(now);
}
void MotorApp_Stop(uint32_t now) {
    if (initialized) stop_and_sample(now,0);
}
bool MotorApp_IsActive(void) { return status.mode==MOTOR_DUTY; }
void MotorApp_KeepAlive(uint32_t now) {
    if (MotorApp_IsActive()) command.timestamp_us=now;
}
const MotorAppStatus *MotorApp_GetStatus(void) { return &status; }
const MotorAppSample *MotorApp_GetSample(void) { return &sample; }
void MotorApp_SetFeedbackCallback(MotorAppFeedbackCallback callback) { feedback_callback=callback; }
bool MotorApp_SetDuty(float left,float right,uint32_t now) {
    if (!request_valid(left,right)) return false;
    command={left,right,now};
    status.mode=MOTOR_DUTY;
    return true;
}
bool MotorApp_ApplyDuty(float left,float right,uint32_t now) {
    if (!request_valid(left,right) || !sample.valid ||
        (uint32_t)(now-sample.timestamp_us)>7500U) return false;
    uint32_t faults=apply_output(left,right);
    if (faults) { stop_and_sample(now,faults); return false; }
    command={left,right,now};
    status.mode=MOTOR_DUTY;
    config.set_driver_enabled(true);
    return true;
}
