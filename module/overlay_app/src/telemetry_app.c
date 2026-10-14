#include "telemetry_app.h"
#include "uart_debug.h"
#include "oled.h"
#include "motor_app.h"
#include "timebase.h"
#include "motion_app.h"
#include <math.h>
#include <limits.h>

//这个文件的任务就是进行串口任务相关的包装，包括想要用上层命令调用什么函数，打印什么东西

static UART_Debug_t *debug;
static int stream = 1;
static bool initialized;

/* 物理量放大为整数遥测，饱和输出避免浮点转整数溢出。 */
static long milli(float value)
{
    float scaled=value*1000;
    if (!isfinite(scaled)) return scaled<0 ? LONG_MIN : LONG_MAX;
    if (scaled>=(float)LONG_MAX) return LONG_MAX;
    if (scaled<=(float)LONG_MIN) return LONG_MIN;
    return (long)scaled;
}

static float motor_left, motor_right;
static void motor_status(void)
{
    const MotorAppStatus *m=MotorApp_GetStatus();
    static const char *names[]={"STOPPED","DUTY","FAULT","UNCONFIGURED"};
    const char *name=(unsigned)m->mode<4 ? names[m->mode] : "INVALID";
    UART_Printf("MOTOR mode=%s us=%lu faults=%lu\r\n", name,
        (unsigned long)m->timestamp_us,(unsigned long)m->faults);
    UART_Printf("WHEEL L=%ld R=%ld mrpm dutyL=%ld dutyR=%ld permille valid=%u,%u\r\n",
        (long)(m->left.rpm*1000),(long)(m->right.rpm*1000),
        (long)(m->left.applied_duty*1000),(long)(m->right.applied_duty*1000),m->left.feedback_valid,m->right.feedback_valid);
    UART_Printf("WHEEL travelL=%ld travelR=%ld mm valid=%u,%u\r\n",
        milli(m->left.displacement_m),milli(m->right.displacement_m),
        m->left.displacement_valid,m->right.displacement_valid);
}
static void motor_stop(void) { MotorApp_Stop(Timebase_NowUs()); motor_left=motor_right=0; UART_Printf("MOTOR STOP\r\n"); }
static void motor_keep(void) { MotorApp_KeepAlive(Timebase_NowUs()); }
static void motor_duty(void){
    bool ok=MotorApp_SetDuty(motor_left,motor_right,Timebase_NowUs());
    if (!ok && MotorApp_IsActive()) MotorApp_Stop(Timebase_NowUs());
    UART_Printf("MOTOR duty %s\r\n",ok ? "OK" : "REJECTED");
}
static void motion_status(void)
{
    const MotionAppStatus *a=MotionApp_GetStatus();
    const MotionState *s=MotionApp_GetState();
    float angles[3]; MotionApp_GetEuler(angles);
    UART_Printf("MPU hz_m=%ld dt_us=%ld ok=%lu missed=%lu nodata=%lu fail=%lu\r\n",
        milli(a->sensor_hz),
        milli(a->last_dt_s*1000),(unsigned long)a->updates,
        (unsigned long)a->missed_periods,(unsigned long)a->no_data,(unsigned long)a->failures);
    UART_Printf("POSE valid=%u trusted=%u gap=%u rpy_mdeg=%ld,%ld,%ld\r\n",
        s->valid,s->accel_trusted,s->discontinuity,
        milli(angles[0]),milli(angles[1]),milli(angles[2]));

}
static void ping(void) { UART_Printf("PONG\r\n"); }
static void status(void)
{
    const SensorStatus *state = SensorApp_GetStatus();
    const ImuSample *sample = SensorApp_GetSample();
    UART_Printf("STATUS ready=%u addr=0x%02X who=0x%02X ok=%lu fail=%lu\r\n",
        state->ready, state->address, state->who_am_i,
        (unsigned long)state->reads, (unsigned long)state->failures);
    UART_Printf("SAMPLE valid=%u us=%lu seq=%lu hal=%u bus=0x%lX\r\n", sample->valid,
        (unsigned long)sample->timestamp_us, (unsigned long)sample->sequence, state->last_result, (unsigned long)state->bus_error);
    UART_Printf("OLED ready=%u addr=0x%02X err=0x%lX chunks=%lu\r\n", OLED_IsReady(),
        OLED_Address(), (unsigned long)OLED_Error(), (unsigned long)OLED_Transfers());
    UART_Printf("UART rx_drop=%lu tx_drop=%lu err=%lu tx_err=%lu\r\n",
        (unsigned long)debug->rx_dropped, (unsigned long)debug->tx_dropped,
        (unsigned long)debug->uart_errors, (unsigned long)debug->tx_errors);
}
bool TelemetryApp_Init(UART_HandleTypeDef *uart){
    //进行串口调试实例的初始化，注册命令表，打印启动信息
    if (initialized) return debug->huart == uart;
    debug = UART_Debug_Init(uart);
    if (!debug) return false;
    UART_CmdEntry_t commands[] = {
        {.cmd="ml", .type=TYPE_FLOAT, .var_ptr=&motor_left},
        {.cmd="mr", .type=TYPE_FLOAT, .var_ptr=&motor_right},
        {.cmd="mdrive", .type=TYPE_NONE, .func=motor_duty},
        {.cmd="mstop", .type=TYPE_NONE, .func=motor_stop},
        {.cmd="mkeep", .type=TYPE_NONE, .func=motor_keep},
        {.cmd="mstatus", .type=TYPE_NONE, .func=motor_status},
        {.cmd="ping", .type=TYPE_NONE, .func=ping},
        {.cmd="status", .type=TYPE_NONE, .func=status},
        {.cmd="stream", .type=TYPE_INT, .var_ptr=&stream},
        {.cmd="pose", .type=TYPE_NONE, .func=motion_status},
    };

    //在这里，将调试命令一条条注册，全都注册成功就将initialized设为true
    for (unsigned i=0; i<sizeof(commands)/sizeof(commands[0]); ++i)
        if (UART_Debug_Register(&commands[i]) != 0) return false;
    initialized = true;
    UART_Printf("BOOT telemetry: IMU / POSE_US / MOTOR; stream 0=off, nonzero=on\r\n");
    return true;
}
void TelemetryApp_Poll(void){
    if (!initialized) return;   //检验初始化状态，失败则返回
    //先检验有没有接收到调试命令，接收到了才去处理命令，触发对应的调试效果，一个优化，避免空转，值得学习
    UART_Debug_t *pending = UART_Debug_Poll();
    if (pending) UART_Debug_Process(pending);
}

/* 三个输出入口互不选择、不自行调度；stream 只作为总开关，写零就表示关掉打印。 */

//打印IMU的矫正数据
void TelemetryApp_ReportImu(const ImuSample *sample, bool available){
    if (!initialized || !stream || !sample || !sample->valid || !available) return;
    UART_Printf("IMU,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)sample->timestamp_us, (unsigned long)sample->sequence,
        milli(sample->data.accel[0]), milli(sample->data.accel[1]), milli(sample->data.accel[2]),
        milli(sample->data.gyro[0]), milli(sample->data.gyro[1]), milli(sample->data.gyro[2]));
}

//打印姿态数据
void TelemetryApp_ReportAttitude(bool available){
    if (!initialized || !stream) return;
    const MotionState *state=MotionApp_GetState();
    uint32_t timestamp=MotionApp_GetStatus()->timestamp_us;
    bool valid=available && state->valid &&
        (uint32_t)(Timebase_NowUs()-timestamp)<IMU_TIMEOUT_US;
    float angles[3];
    MotionApp_GetEuler(angles);
    /* 无效时仍发 valid=0，角度仅为保留值，不冒充新鲜结果。 */
    UART_Printf("POSE_US,%lu,%u,%ld,%ld,%ld\r\n",
        (unsigned long)timestamp,valid,milli(angles[0]),milli(angles[1]),milli(angles[2]));
}

//打印两轮速度和位移数据
void TelemetryApp_ReportMotor(const MotorAppStatus *motor){
    if (!initialized || !stream || !motor) return;
    /* 停止状态也输出，便于观察手动转轮；无效数值由对应标志指明。 */
    UART_Printf("MOTOR,%luus,%u,l %ld rpm,r %ld rpm,l %ld d,r %ld d,%u,%u,%u,%u\r\n",
        (unsigned long)motor->timestamp_us,(unsigned)motor->mode,
        milli(motor->left.rpm),milli(motor->right.rpm),
        milli(motor->left.displacement_m),milli(motor->right.displacement_m),
        motor->left.feedback_valid,motor->right.feedback_valid,
        motor->left.displacement_valid,motor->right.displacement_valid);
}
