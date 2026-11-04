#ifndef TRACE_SOURCES_H
#define TRACE_SOURCES_H
#include "trace_app.h"
#define LQR_TRACE_CAPACITY 64U
#define IMU_TRACE_CAPACITY 24U
#define TRACE_SOURCE_LQR 1U
#define TRACE_SOURCE_IMU 2U
/* 载荷和数组均有明确类型；不使用统一的最大载荷字节数组。 */
typedef struct {
    uint32_t sample_timestamp_us, sample_sequence, sample_valid;
    uint32_t input_timestamp_us, input_sequence, dt_us, age_us;
    float error[6], torque_nm[2], duty[2], relative_rpm[2], pitch_rate_scale, yaw_rate_scale;
} LqrTracePayload;
typedef struct {
    uint32_t sample_timestamp_us, sample_sequence, sample_valid;
    float accel[3], gyro[3];
} ImuTracePayload;
typedef struct { TraceHeader header; LqrTracePayload data; } LqrTraceFrame;
typedef struct { TraceHeader header; ImuTracePayload data; } ImuTraceFrame;
/* 静态分配或 = {0}；每个对象包含自己的数组和源变量，禁止初始化后复制。 */
typedef struct {
    Trace recorder;
    LqrTraceFrame frames[LQR_TRACE_CAPACITY];
    LqrTracePayload source;
} LqrTrace;
typedef struct {
    Trace recorder;
    ImuTraceFrame frames[IMU_TRACE_CAPACITY];
    ImuTracePayload source;
} ImuTrace;
bool LqrTrace_Init(LqrTrace *trace);
bool ImuTrace_Init(ImuTrace *trace);
bool LqrTrace_Read(const LqrTrace *trace, uint32_t index, LqrTraceFrame *out);
bool ImuTrace_Read(const ImuTrace *trace, uint32_t index, ImuTraceFrame *out);
/* 统一管理：先准备源变量，再调用通用记录器；不控制被观察业务。 */
void TraceSources_Poll(Trace *trace, uint32_t now_us);
void TraceSources_Freeze(Trace *trace, uint32_t now_us);
#endif
