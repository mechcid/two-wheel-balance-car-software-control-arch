#include "telemetry_app.h"
#include "uart_debug.h"
#include "oled.h"
#include "motor_app.h"
#include "timebase.h"
#include "motion_app.h"
#include <math.h>
#include <limits.h>

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
        (long)(m->left.duty*1000),(long)(m->right.duty*1000),m->left.feedback_valid,m->right.feedback_valid);
}
static void motor_stop(void) { MotorApp_Stop(Timebase_NowUs()); motor_left=motor_right=0; UART_Printf("MOTOR STOP\r\n"); }
static void motor_keep(void) { MotorApp_KeepAlive(Timebase_NowUs()); }
static void motor_duty(void)
{
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
bool TelemetryApp_Init(UART_HandleTypeDef *uart)
{
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
    for (unsigned i=0; i<sizeof(commands)/sizeof(commands[0]); ++i)
        if (UART_Debug_Register(&commands[i]) != 0) return false;
    initialized = true;
    UART_Printf("BOOT IMU: timestamp_us,sequence,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps\r\n");
    return true;
}
void TelemetryApp_Poll(void)
{
    if (!initialized) return;
    UART_Debug_t *pending = UART_Debug_Poll();
    if (pending) UART_Debug_Process(pending);
}
void TelemetryApp_Report(const ImuSample *sample, bool available)
{
    if (!initialized || !stream || !sample || !sample->valid || !available) return;
    if (stream==2) {
        const MotionState *s=MotionApp_GetState();
        float angles[3]; MotionApp_GetEuler(angles);
        UART_Printf("POSE_US,%lu,%u,%ld,%ld,%ld\r\n",
            (unsigned long)MotionApp_GetStatus()->timestamp_us,s->valid,
            milli(angles[0]),milli(angles[1]),milli(angles[2]));
        return;
    }
    /* 将物理量放大后以整数输出，避免引入浮点 printf；用新标记区分协议。 */
    UART_Printf("IMU,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)sample->timestamp_us, (unsigned long)sample->sequence,
        (long)(sample->data.accel[0]*1000), (long)(sample->data.accel[1]*1000), (long)(sample->data.accel[2]*1000),
        (long)(sample->data.gyro[0]*1000),
        (long)(sample->data.gyro[1]*1000), (long)(sample->data.gyro[2]*1000));
}
