#include "telemetry_app.h"
#include "uart_debug.h"
#include "oled.h"

static UART_Debug_t *debug;
static int stream = 1;
static bool initialized;
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
