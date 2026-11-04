#ifndef TRACE_APP_H
#define TRACE_APP_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* 多实例、每个实例由主循环单写者。禁止 ISR 写入；volatile 不提供并发/原子性保证。
 * trace 只用来观察，调试业务状态，不读取传感器、不控制电机，也不能使断点停机安全。 */

//记录模块状态：空闲、记录中、冻结、等待业务启动。
typedef enum { 
    TRACE_IDLE, 
    TRACE_RECORDING, 
    TRACE_FROZEN,
    TRACE_ARMED /* 已准备，等待业务开始及第一份数据 */
} TraceMode;

/* 适配层解释业务状态，通用记录器只理解这些记录条件。 */
typedef enum {
    TRACE_SOURCE_STOPPED, 
    TRACE_SOURCE_NORMAL, 
    TRACE_SOURCE_FAULT,
    TRACE_SOURCE_WAITING, 
    TRACE_SOURCE_INVALID
} TraceSourceState;
//定义记录的是什么东西，业务模块正常采样的数据，停止，故障，数据违法，手动冻结
typedef enum { 
    TRACE_SAMPLE, 
    TRACE_STOP_EVENT, 
    TRACE_FAULT_EVENT, 
    TRACE_INVALID_EVENT, 
    TRACE_MANUAL_EVENT 
} TraceEvent;

//定义记录模块为什么冻结
typedef enum { 
    TRACE_FREEZE_NONE, 
    TRACE_FREEZE_STOP, 
    TRACE_FREEZE_FAULT, 
    TRACE_FREEZE_INVALID, 
    TRACE_FREEZE_MANUAL 
} TraceFreezeReason;

typedef struct {
    TraceSourceState state;
    uint32_t faults;
} TraceSourceStatus;
typedef TraceSourceStatus (*TraceCheckSource)(void *context);

//每种类型记录共有的头部；具体帧必须以此成员开头。
typedef struct {
    uint32_t timestamp_us, sequence, business_mode, faults, event, valid;
} TraceHeader;

//用于记录 记录模块的工作状态
typedef struct {
    TraceMode mode;
    TraceFreezeReason freeze_reason;
    uint32_t source_id, format_id, payload_size, session;
    uint32_t count, write_index, total_written;
    bool business_started; /* ARMED 曾观察到运行，随后停止也应保留终止事件。 */
} TraceStatus;
/* 检测函数只查询状态，不采集硬件、不控制业务、不修改源数据。
 * 源变量由外部在 Poll/Freeze 前更新，与数组均须持续有效且不重叠。 */
typedef struct {
    uint32_t source_id, format_id, payload_size;
    const void *source;
    size_t source_bytes;
    uint32_t frequency_hz;
    TraceCheckSource check_source;
    void *context;
    void *storage;           /* 独占的有类型帧数组，生命周期覆盖实例。 */
    size_t storage_bytes, frame_size, payload_offset;
    uint32_t capacity;
} TraceConfig;
/* 对象必须零初始化；初始化后禁止复制对象或共享其存储。 */
typedef struct {
    volatile TraceStatus status;
    TraceConfig config;
    uint32_t last_record_us, period_us;
    bool initialized;
} Trace;
/* 每个实例在 main 初始化一次，只绑定来源，不开始记录。 */
bool TraceApp_Init(Trace *trace, const TraceConfig *config);
/* 唯一清空入口，IDLE/FROZEN -> ARMED；等待期间停止不冻结。 */
bool TraceApp_Arm(Trace *trace);
/* 每轮检查业务状态，正常时按频率复制；延迟时仅记录一次，不补历史。
 * 头部时间是观察时刻，序号是本次记录会话的记录序号；业务身份留在载荷。
 * 终止时绕过周期限制保存当前源变量，再冻结。 */
void TraceApp_Poll(Trace *trace, uint32_t now_us);
/* 只冻结记录，不停机。 */
void TraceApp_Freeze(Trace *trace, uint32_t now_us);
const volatile TraceStatus *TraceApp_GetStatus(const Trace *trace);
/* 只允许冻结后按时间顺序复制读取；避免得到正在改写的半帧。 */
bool TraceApp_Read(const Trace *trace, uint32_t chronological_index, void *out, size_t out_size);
#endif
