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

//用于要记录业务模块的工作状态，停止，运行，内部出错
typedef enum { 
    TRACE_BUSINESS_STOPPED, 
    TRACE_BUSINESS_RUNNING, 
    TRACE_BUSINESS_FAULT 
} TraceBusinessMode;

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

/* 由业务模块从自己的状态结构体生成；valid 表示本帧载荷可用。 */
//业务模块与记录模块的接口，定义业务模块是否正在工作，数据的合法性。
//时间戳，序列号，是否错误
typedef struct {
    TraceBusinessMode mode;
    bool valid;
    bool has_sample; /* 尚未产生第一份数据不同于已产生无效数据。 */
    uint32_t timestamp_us, sequence, faults;
} TraceBusinessStatus;

//每种类型记录共有的头部；具体帧必须以此成员开头。
typedef struct {
    uint32_t timestamp_us, sequence, business_mode, faults, event, valid;
} TraceHeader;

//用于记录 记录模块的工作状态
typedef struct {
    TraceMode mode;
    TraceFreezeReason freeze_reason;
    uint32_t source_id, format_id, payload_size, session;
    uint32_t count, write_index, total_written, duplicate_frames;
    uint32_t last_sequence;
    bool have_sequence;
    bool business_started; /* ARMED 曾观察到运行，随后停止也应保留终止事件。 */
} TraceStatus;
/* 同一主循环内一次读取一致的快照；不得采集硬件、控制电机或保留 payload 指针。
 * 返回 false 表示快照读取失败，将生成无效事件并冻结。 */
typedef bool (*TraceReadSnapshot)(void *context, TraceBusinessStatus *business,
                                  void *payload, uint32_t capacity);
typedef struct {
    uint32_t source_id, format_id, payload_size;
    TraceReadSnapshot read_snapshot;
    void *context;
    void *storage;           /* 独占的有类型帧数组，生命周期覆盖实例。 */
    size_t storage_bytes, frame_size, payload_offset;
    uint32_t capacity;
    void *scratch;           /* 与载荷同类型的独立临时对象，不能与数组重叠。 */
    size_t scratch_bytes;
} TraceConfig;
/* 对象必须零初始化；初始化后禁止复制对象或共享其存储。 */
typedef struct {
    volatile TraceStatus status;
    TraceConfig config;
    TraceBusinessMode last_business_mode;
    bool initialized;
} Trace;
/* 每个实例在 main 初始化一次，只绑定来源，不开始记录。 */
bool TraceApp_Init(Trace *trace, const TraceConfig *config);
/* 唯一清空入口，IDLE/FROZEN -> ARMED；等待期间停止不冻结。 */
bool TraceApp_Arm(Trace *trace);
/* 放在业务更新之后。正常帧使用业务数据自己的时间戳和序号。
 * now_us 用于终止事件，表示轮询发现状态变化的时间。 */
void TraceApp_Poll(Trace *trace, uint32_t now_us);
/* 只冻结记录，不停机。 */
void TraceApp_Freeze(Trace *trace, uint32_t now_us);
const volatile TraceStatus *TraceApp_GetStatus(const Trace *trace);
/* 只允许冻结后按时间顺序复制读取；避免得到正在改写的半帧。 */
bool TraceApp_Read(const Trace *trace, uint32_t chronological_index, void *out, size_t out_size);
#endif
