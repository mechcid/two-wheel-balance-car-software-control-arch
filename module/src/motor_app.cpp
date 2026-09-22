#include "motor_app.h"
#include "motor_speed_controller.hpp"
#include <new>

namespace {
Encoder encoders[2]{};
alignas(Motor) unsigned char motor_storage[2][sizeof(Motor)];
alignas(MotorSpeedController) unsigned char control_storage[2][sizeof(MotorSpeedController)];
Motor *motors[2]{};
MotorSpeedController *controllers[2]{};
MotorAppConfig config{};
MotorAppStatus status{MOTOR_UNCONFIGURED, {}, {}, {}, {}, 0, 0};
uint32_t last_update, last_command;
bool initialized;
constexpr uint32_t command_timeout_ms = 1000;
constexpr uint32_t max_control_gap_ms = 50;

void snapshot(uint32_t now)
{
    for (unsigned i=0; i<2; ++i) {
        status.rpm[i] = motors[i]->state().rpm;
        status.duty[i] = motors[i]->state().applied_duty;
        status.feedback_valid[i] = motors[i]->state().feedback_valid;
    }
    status.timestamp_ms = now;
}
void fault(uint32_t now)
{
    MotorApp_Stop(now);
    status.mode = MOTOR_FAULT;
    status.faults++;
}
bool request_valid(float left, float right, float limit)
{
    return initialized && status.mode != MOTOR_FAULT &&
        std::isfinite(left) && std::isfinite(right) &&
        std::fabs(left) <= limit && std::fabs(right) <= limit;
}
}

bool MotorApp_Init(const MotorAppConfig *settings, uint32_t now)
{
    if (initialized || motors[0]) return false; // 仅启动时初始化一次，避免重复构造硬件所有者。
    if (!settings || !std::isfinite(settings->duty_limit) || settings->duty_limit <= 0 ||
        settings->duty_limit > 1 || !std::isfinite(settings->rpm_limit) || settings->rpm_limit <= 0)
        return false;
    config = *settings;
    if (!config.set_driver_enabled) return false;
    config.set_driver_enabled(false);
    for (unsigned i=0; i<2; ++i) {
        const auto &wheel = config.wheels[i];
        encoders[i].timer = wheel.encoder_timer;
        MotorConfig c{};
        c.direction1=wheel.direction1; c.direction2=wheel.direction2; c.pwm=wheel.pwm;
        c.encoder=&encoders[i]; c.counts_per_output_rev=wheel.counts_per_rev;
        c.output_reversed=wheel.output_reversed; c.encoder_reversed=wheel.encoder_reversed;
        c.duty_limit=config.duty_limit;
        c.forward_in1=config.forward_in1; c.forward_in2=config.forward_in2;
        c.reverse_in1=config.reverse_in1; c.reverse_in2=config.reverse_in2;
        c.idle_in1=config.idle_in1; c.idle_in2=config.idle_in2;
        motors[i]=new (motor_storage[i]) Motor(c);
        controllers[i]=new (control_storage[i]) MotorSpeedController(*motors[i]);
        PID::Config pid;
        pid.mode=wheel.incremental_pid ? PID::Mode::Incremental : PID::Mode::Position;
        pid.kp=wheel.kp; pid.ki=wheel.ki; pid.kd=wheel.kd;
        pid.output_min=pid.integral_min=-config.duty_limit;
        pid.output_max=pid.integral_max=config.duty_limit;
        if (!motors[i]->begin() || !controllers[i]->configure(pid)) {
            for (unsigned j=0; j<=i; ++j) motors[j]->stop();
            status.mode=MOTOR_FAULT;
            return false;
        }
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
    config.set_driver_enabled(false);
    bool ok=true;
    for (unsigned i=0; i<2; ++i) {
        ok=controllers[i]->disable() && ok;
        status.duty[i]=motors[i]->state().applied_duty;
        status.target[i]=0;
        ok=motors[i]->rebaseFeedback() && ok;
    }
    status.mode=ok ? MOTOR_STOPPED : MOTOR_FAULT;
    last_update=now;
    snapshot(now);
}
bool MotorApp_IsActive(void)
{ return status.mode==MOTOR_DUTY || status.mode==MOTOR_SPEED; }
void MotorApp_KeepAlive(uint32_t now)
{ if (MotorApp_IsActive()) last_command=now; }
const MotorAppStatus *MotorApp_GetStatus(void) { return &status; }

bool MotorApp_SetDuty(float left, float right, uint32_t now)
{
    if (!request_valid(left,right,config.duty_limit)) return false;
    if (status.mode!=MOTOR_DUTY) MotorApp_Stop(now);
    if (status.mode==MOTOR_FAULT) return false;
    status.target[0]=left; status.target[1]=right;
    status.mode=MOTOR_DUTY;
    last_command=now;
    // 输出在下一个调度周期统一写入，命令回调不直接启动电机。
    return true;
}
bool MotorApp_SetRPM(float left, float right, uint32_t now)
{
    if (!config.speed_control_allowed || !request_valid(left,right,config.rpm_limit)) return false;
    for (const auto &wheel:config.wheels)
        if (wheel.kp==0 && wheel.ki==0 && wheel.kd==0) return false;
    bool was_active=status.mode==MOTOR_SPEED;
    if (!was_active) MotorApp_Stop(now);
    if (status.mode==MOTOR_FAULT) return false;
    if (!controllers[0]->setTargetRPM(left) || !controllers[1]->setTargetRPM(right) ||
        !controllers[0]->enable() || !controllers[1]->enable()) { fault(now); return false; }
    status.target[0]=left; status.target[1]=right;
    if (!was_active) last_update=now;
    last_command=now;
    status.mode=MOTOR_SPEED;
    return true;
}
void MotorApp_Update(uint32_t now)
{
    if (!initialized) return;
    uint32_t elapsed=now-last_update;
    if (!elapsed) return;
    bool active=MotorApp_IsActive();
    if (active && (elapsed>max_control_gap_ms || (uint32_t)(now-last_command)>=command_timeout_ms)) {
        fault(now); return;
    }
    last_update=now;
    if (status.mode==MOTOR_FAULT) return;
    float dt=elapsed*0.001f;
    for (unsigned i=0; i<2; ++i) {
        bool ok;
        if (status.mode==MOTOR_SPEED) ok=controllers[i]->update(dt);
        else {
            ok=motors[i]->updateFeedback(dt);
            if (ok && status.mode==MOTOR_DUTY) ok=motors[i]->setDuty(status.target[i]);
        }
        if (!ok) { fault(now); break; }
    }
    if (MotorApp_IsActive()) config.set_driver_enabled(true);
    snapshot(now);
}
