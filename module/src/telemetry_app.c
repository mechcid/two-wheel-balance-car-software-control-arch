#include "telemetry_app.h"
#include "uart_debug.h"
#include "oled.h"
#include "motor_app.h"

static UART_Debug_t *debug;
static int stream = 1;
static bool initialized;

static float motor_left, motor_right;
static void motor_status(void)
{
    const MotorAppStatus *m=MotorApp_GetStatus();
    static const char *names[]={"STOPPED","DUTY","SPEED","FAULT","UNCONFIGURED"};
    const char *name=(unsigned)m->mode<5 ? names[m->mode] : "INVALID";
    UART_Printf("MOTOR mode=%s tick=%lu faults=%lu\r\n", name,
        (unsigned long)m->timestamp_ms,(unsigned long)m->faults);
    UART_Printf("WHEEL L=%ld R=%ld mrpm dutyL=%ld dutyR=%ld permille valid=%u,%u\r\n",
        (long)(m->rpm[0]*1000),(long)(m->rpm[1]*1000),
        (long)(m->duty[0]*1000),(long)(m->duty[1]*1000),m->feedback_valid[0],m->feedback_valid[1]);
}
static void motor_stop(void) { MotorApp_Stop(HAL_GetTick()); motor_left=motor_right=0; UART_Printf("MOTOR STOP\r\n"); }
static void motor_keep(void) { MotorApp_KeepAlive(HAL_GetTick()); }
static void motor_duty(void)
{
    bool ok=MotorApp_SetDuty(motor_left,motor_right,HAL_GetTick());
    if (!ok && MotorApp_IsActive()) MotorApp_Stop(HAL_GetTick());
    UART_Printf("MOTOR duty %s\r\n",ok ? "OK" : "REJECTED");
}
static void motor_speed(void)
{
    bool ok=MotorApp_SetRPM(motor_left,motor_right,HAL_GetTick());
    if (!ok && MotorApp_IsActive()) MotorApp_Stop(HAL_GetTick());
    UART_Printf("MOTOR speed %s\r\n",ok ? "OK" : "REJECTED");
}

static void ping(void) { UART_Printf("PONG\r\n"); }
static void status(void)
{
    const SensorStatus *state = SensorApp_GetStatus();
    const SensorSample *sample = SensorApp_GetSample();
    UART_Printf("STATUS ready=%u addr=0x%02X who=0x%02X ok=%lu fail=%lu\r\n",
        state->ready, state->address, state->who_am_i,
        (unsigned long)state->reads, (unsigned long)state->failures);
    UART_Printf("SAMPLE valid=%u tick=%lu hal=%u bus=0x%lX\r\n", sample->valid,
        (unsigned long)sample->timestamp_ms, state->last_result, (unsigned long)state->bus_error);
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
        {.cmd="mrun", .type=TYPE_NONE, .func=motor_speed},
        {.cmd="mstop", .type=TYPE_NONE, .func=motor_stop},
        {.cmd="mkeep", .type=TYPE_NONE, .func=motor_keep},
        {.cmd="mstatus", .type=TYPE_NONE, .func=motor_status},
        {.cmd="ping", .type=TYPE_NONE, .func=ping},
        {.cmd="status", .type=TYPE_NONE, .func=status},
        {.cmd="stream", .type=TYPE_INT, .var_ptr=&stream},
    };
    for (unsigned i=0; i<sizeof(commands)/sizeof(commands[0]); ++i)
        if (UART_Debug_Register(&commands[i]) != 0) return false;
    initialized = true;
    UART_Printf("BOOT MPU_C: tick,ax_mg,ay_mg,az_mg,temp_mC,gx_mdps,gy_mdps,gz_mdps\r\n");
    return true;
}
void TelemetryApp_Poll(void)
{
    if (!initialized) return;
    UART_Debug_t *pending = UART_Debug_Poll();
    if (pending) UART_Debug_Process(pending);
}
void TelemetryApp_Report(const SensorSample *sample, bool available)
{
    if (!initialized || !stream || !sample || !sample->valid || !available) return;
    /* 将物理量放大后以整数输出，避免引入浮点 printf；用新标记区分协议。 */
    UART_Printf("MPU_C,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)sample->timestamp_ms,
        (long)(sample->ax_g*1000), (long)(sample->ay_g*1000), (long)(sample->az_g*1000),
        (long)(sample->temperature_c*1000), (long)(sample->gx_dps*1000),
        (long)(sample->gy_dps*1000), (long)(sample->gz_dps*1000));
}
