#include "motor_app.h"
#include "motor.hpp"
#include <new>

namespace {//匿名命名空间，作用类似于static，花括号内的内容只在当前文件内部可见
alignas(Motor) unsigned char left_motor_storage[sizeof(Motor)];
alignas(Motor) unsigned char right_motor_storage[sizeof(Motor)];
Motor *left_motor{};
Motor *right_motor{};
struct MotorCommand {
    float left_duty;
    float right_duty;
};
MotorCommand command{};
MotorAppConfig config{};
MotorAppStatus status{MOTOR_UNCONFIGURED, {}, {}, 0, 0, 0, 0, false};
MotorAppFeedbackCallback feedback_callback{};
uint32_t last_update, last_command;
bool initialized;
constexpr uint32_t command_timeout_us = 1000000;
constexpr uint32_t max_control_gap_us = 50000;

bool stop_motor(Motor &motor, float dt_s){
    bool stopped = motor.stop();
    bool rebased = motor.rebaseFeedback(dt_s);
    return stopped && rebased;
}

//用于更新记录两个电机的状态，并且给更新的数据打上时间戳记录，输出到串口
//本身并不参与电机逻辑上的组织
void snapshot(uint32_t now)
{
    status.left = left_motor->state();
    status.right = right_motor->state();
    status.timestamp_us = now;
}
// 每次编码器采集只发布一次；回调在下一份数据覆盖前同步消费。
void publish(uint32_t now, uint32_t elapsed, bool valid){
    snapshot(now);
    status.interval_us = elapsed;
    status.valid = valid && elapsed > 0 && elapsed <= max_control_gap_us &&
        status.left.delta_valid && status.right.delta_valid;
    ++status.sequence;
    if (feedback_callback) feedback_callback(&status);
}
void stop_and_sample(uint32_t now, bool force_fault)
{
    config.set_driver_enabled(false);
    uint32_t elapsed = now - last_update;
    float dt_s=(elapsed>0 && elapsed<=max_control_gap_us) ? elapsed*0.000001f : 0;
    bool left_ok = stop_motor(*left_motor, dt_s);
    bool right_ok = stop_motor(*right_motor, dt_s);
    command = {};
    status.mode = (left_ok && right_ok && !force_fault) ? MOTOR_STOPPED : MOTOR_FAULT;
    if (force_fault) ++status.faults;
    last_update = now;
    publish(now, elapsed, true);
}
bool request_valid(float left, float right)
{
    return initialized && status.mode != MOTOR_FAULT &&
        std::isfinite(left) && std::isfinite(right) &&
        std::fabs(left) <= config.left.duty_limit && std::fabs(right) <= config.right.duty_limit;
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
        return false;
    config = *settings;
    config.set_driver_enabled(false);
    left_motor = new (left_motor_storage) Motor(config.left);
    if (!left_motor->begin()) {
        left_motor->stop();
        status.mode=MOTOR_FAULT;
        return false;
    }
    right_motor = new (right_motor_storage) Motor(config.right);
    if (!right_motor->begin()) {
        left_motor->stop();
        right_motor->stop();
        status.mode=MOTOR_FAULT;
        return false;
    }
    initialized=true;
    last_update=last_command=now;
    status.mode=MOTOR_STOPPED;
    snapshot(now);
    return true;
}
void MotorApp_Stop(uint32_t now)
{
    if (!initialized) return;
    stop_and_sample(now, false);
}
bool MotorApp_IsActive(void){
    return status.mode==MOTOR_DUTY; }

void MotorApp_KeepAlive(uint32_t now){
    if (MotorApp_IsActive()) last_command=now; }

const MotorAppStatus *MotorApp_GetStatus(void) { return &status; }
void MotorApp_SetFeedbackCallback(MotorAppFeedbackCallback callback) { feedback_callback = callback; }

//暴露出来用于控制两个电机占空比的函数
bool MotorApp_SetDuty(float left, float right, uint32_t now){
    if (!request_valid(left,right)) return false;
    // STOPPED 已关闭输出，启动只提交命令，不额外消耗编码器采样。
    if (status.mode==MOTOR_FAULT) return false;
    command.left_duty=left; command.right_duty=right;
    status.mode=MOTOR_DUTY;
    last_command=now;
    // 输出在下一个调度周期统一写入，命令回调不直接启动电机。
    return true;
}

//更新函数，传入一个时间基准就能自动更新计算出速度
//速度计算是要时刻更新的，但是占空比设置相对于速度计算独立，写个语句就能设置
void MotorApp_Update(uint32_t now){
    if (!initialized) return;
    if (status.mode==MOTOR_FAULT) return;
    uint32_t elapsed=now-last_update;
    if (!elapsed) return;
    bool active=MotorApp_IsActive();
    if (active && (elapsed>max_control_gap_us || (uint32_t)(now-last_command)>=command_timeout_us)) {
        stop_and_sample(now, true);
        return;
    }
    last_update=now;
    float dt=elapsed*0.000001f;
    // 两轮都采样，不能因左轮失败而跳过右轮、造成采样区间错位。
    bool left_ok = left_motor->updateFeedback(dt);
    bool right_ok = right_motor->updateFeedback(dt);
    bool feedback_ok = left_ok && right_ok;
    bool output_ok = true;
    if (feedback_ok && active) {
        bool left_output = left_motor->setDuty(command.left_duty);
        bool right_output = right_motor->setDuty(command.right_duty);
        output_ok = left_output && right_output;
    }
    if (!feedback_ok || !output_ok) {
        config.set_driver_enabled(false);
        left_motor->stop();
        right_motor->stop();
        command = {};
        status.mode = MOTOR_FAULT;
        ++status.faults;
        // 故障停机只关闭输出，不再次采集并覆盖刚得到的增量。
    } else if (active) {
        config.set_driver_enabled(true);
    }
    publish(now, elapsed, feedback_ok);
}
