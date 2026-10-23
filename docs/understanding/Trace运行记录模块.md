# Trace 运行记录模块：有类型的多实例设计

Trace 使用 C 的对象式接口：每个对象独立持有存储、状态和回调，所有对象共用环形管理函数。LQR 与 IMU 不共用数组，可以同时记录、分别冻结和重新准备。没有动态内存分配。

## 文件与职责

| 文件 | 职责 |
|---|---|
| trace_app.h/.c | 通用 Trace 对象、状态机、去重、环形写入、冻结与读取；不依赖业务类型 |
| trace_sources.h/.c | LqrTrace、ImuTrace 的具体数据类型、数组、快照适配、带类型初始化和读取接口 |
| debug_app.h/.c | 接收 main 创建的对象；轮询两者，协调七秒 LQR 试验和调试系数恢复 |
| main.c | 静态创建两个实例，初始化并注入 Debug |

业务模块继续独立维护状态，不调用 Trace。适配器只读取业务已发布的数据，不重新采集硬件，不执行控制。

## 对象组成与调用示例

实际类型定义在 trace_sources.h，简化后如下：

```c
typedef struct { TraceHeader header; LqrTracePayload data; } LqrTraceFrame;
typedef struct { TraceHeader header; ImuTracePayload data; } ImuTraceFrame;

typedef struct {
    Trace recorder;
    LqrTraceFrame frames[LQR_TRACE_CAPACITY];
    LqrTracePayload scratch;
} LqrTrace;
```

`recorder` 是通用管理对象，`frames` 是这个对象独占的数组，`scratch` 是读取最新快照的临时载荷。IMU 采用相同组成，但数组和临时载荷是 IMU 类型。

main 中初始化：

```c
static LqrTrace lqr_trace;
static ImuTrace imu_trace;

if (!LqrTrace_Init(&lqr_trace) || !ImuTrace_Init(&imu_trace)) Error_Handler();
if (!DebugApp_Init(&lqr_trace, &imu_trace)) Error_Handler();
```

静态对象自动零初始化。若使用其他存储方式，对象也必须先零初始化。初始化之后不能复制对象，因为其中的配置指针指向对象自身的数组与临时载荷。所有存储必须持续有效，不能让两个实例共享数组或 scratch。

操作显式指定对象：

```c
TraceApp_Arm(&lqr_trace.recorder);
TraceApp_Arm(&imu_trace.recorder);
// main 中由 DebugApp_Poll(now) 轮询两者；独立使用时也可以直接调用：
TraceApp_Poll(&lqr_trace.recorder, now);
TraceApp_Poll(&imu_trace.recorder, now);
TraceApp_Freeze(&imu_trace.recorder, now);
ImuTraceFrame frame;
if (ImuTrace_Read(&imu_trace, 0, &frame)) {
    // frame.header.timestamp_us、frame.data.accel[0] 等均有明确类型。
}
```

冻结其中一个不会冻结另一个。上面的 Arm/Poll/Freeze 都不会启动或停止电机。

## 通用内核如何管理不同类型

`TraceConfig` 保存来源 ID、格式版本、载荷大小、快照回调、上下文，以及数组地址、数组字节数、帧大小、载荷偏移、容量和临时载荷地址。

带类型初始化函数用 `sizeof` 和 `offsetof` 自动填写这些信息。内核按字节地址定位数组元素，通过 memcpy 复制头部和载荷，不把所有来源强行解释成某个统一载荷结构。初始化检查容量、数组长度、载荷范围和临时空间长度；通用读取还检查输出缓冲区大小。

C 的通用内核仍不能验证调用方是否选错了数组类型，因此业务代码优先使用 `LqrTrace_Init/Read`、`ImuTrace_Init/Read` 包装接口。新增来源需要定义对应的帧、对象和适配函数，不需要复制环形管理代码。

## 状态与有效性

每个实例独立保存 mode、freeze_reason、source_id、format_id、payload_size、session、count、write_index、total_written、duplicate_frames、last_sequence、have_sequence 和 business_started。

- Init：绑定存储和来源，进入 IDLE，不开始记录。
- Arm：从 IDLE/FROZEN 进入 ARMED，重置元数据并增加会话号。不会清空整个数组；count 决定本会话有效范围。
- ARMED：业务尚未启动时等待；业务已运行但尚无第一份数据时也等待。
- RECORDING：正常且有效的新序号写入，重复序号跳过。
- FROZEN：不再读取回调或改写数组。业务恢复不会自动解冻，必须显式 Arm。

业务快照含 mode、valid、has_sample、timestamp_us、sequence、faults。has_sample=false 表示尚未产生第一份数据，区别于已经产生但无效的数据。

故障优先于停止和数据有效性判断。停止（已经观察到业务运行后）、故障、无效数据或回调失败都会追加对应事件并冻结。事件 valid=0，载荷清零，不保留上一份测量作为事件数据。手动冻结记录最后观察到的业务模式，不会假定业务必然运行。

业务负责 NaN、新鲜度和反馈匹配等检查。Trace 只依照业务报告做记录决策。

## 时间、序号和轮询

正常记录保留来源自己的时间戳和序号，终止事件使用轮询发现或手动冻结的时间。

- IMU 使用 ImuSample 的时间和序号；当前时间是软件读取开始时刻，并不是 MPU6050 内部精确采样时刻。
- LQR 使用控制更新的时间与独立输出序号，并在载荷里保留输入状态时间、输入序号。控制更新时间不是 PWM 边沿测量。
- LQR 轮速与调试系数来自该次控制输出保存的快照，不拼接后来更新的电机反馈。

去重只比较序号是否相等，支持 uint32 自然回绕。duplicate_frames 是重复轮询计数，并非传感器重复采样次数。轮询只读取最新快照，两次轮询之间的中间数据不会自动补回。

当前所有更新、快照和写入在主循环串行执行。main 在业务更新后和命令处理后调用 DebugApp_Poll，由它统一轮询；已经去掉 main 再额外直接调用 Trace 的重复路径。ISR 不得并发写入，volatile 不提供原子性。未来引入中断发布数据时，需要另外解决快照一致性。

## 环形缓冲区和内存

write_index 指向下一次写入位置，count 饱和到容量。满后覆盖最旧记录。未满时最旧记录在 0，满时在 write_index。Read 按 `(oldest+index)%capacity` 返回从旧到新的完整帧，只允许冻结后读取。

| 实例 | 帧大小 | 容量 | 数组占用 | 200 Hz 下窗口 |
|---|---:|---:|---:|---:|
| LQR | 96 字节 | 64 | 6144 字节 | 约 0.32 秒 |
| IMU | 48 字节 | 24 | 1152 字节 | 约 0.12 秒 |

两个数组合计 7296 字节，另有每个对象的管理信息与 scratch。终止事件也占一槽。容量在 trace_sources.h 中分别调整，增大后必须重新检查 RAM 及栈余量。

本次 Debug 固件链接 RAM=16128 字节（78.75%）、Flash=54904 字节（83.78%）。这些是链接占用，不是运行时栈峰值测量。

LQR 来源 source=1、format=3，载荷 72 字节；IMU 来源 source=2、format=1，载荷 24 字节。LQR 包含输入时间和序号、dt/age、六维误差、两轮力矩/占空比/相对转速及 pitch_rate_scale、yaw_rate_scale。IMU 包含三轴加速度（g）和角速度（度/秒）。

## 命令和导出

命令以 CRLF 结束：

| 命令 | 行为 |
|---|---|
| @tracearm | 准备 LQR 记录，等待业务启动，不启动电机 |
| @imutracearm | 独立准备 IMU 记录 |
| @imutracefreeze | 手动冻结 IMU 记录，不停止传感器或电机 |
| @tracestatus | 分别打印 LQR 与 IMU 的状态、容量和计数，以 source 区分 |
| @lqrtrial / @lqrsoft | 启动 LQR 和 LQR 新会话，七秒自动停机，不影响 IMU 记录状态 |
| @mstop | 停止 LQR，并保留其故障或停止事件；IMU 继续按自身状态记录 |

`@lqrstart` 只启动业务，不自动 Arm。七秒试验要求 LQR Trace 处于 IDLE/FROZEN，不要预先 tracearm。`@lqrsoft` 固定使用 0.70，结束或故障后恢复原值；默认系数仍为 0.5，当前 K 保持用户设置。

Trace mode：0 IDLE、1 RECORDING、2 FROZEN、3 ARMED。冻结原因：0 无、1 停止、2 故障、3 无效、4 手动。

调试器中可分别查看 main.c 的 `lqr_trace.frames`、`imu_trace.frames` 和各自 `recorder.status`。实际数组顺序可能回绕，不能直接认为 frames[0] 永远最旧。

导出脚本仍为 tools/export_lqr_trace.gdb，通过环境变量 TRACE_SOURCE=lqr（默认）或 imu 选择实例。LQR_TRACE_OUT 兼容原路径覆盖变量；默认输出 docs/experiments/lqr_trace.json 或 imu_trace.json。输出保持 schema_version=3，不导出事件载荷，并拒绝覆盖已有文件。

必须在连接会暂停 CPU 的调试器之前停止电机；冻结 Trace 不等于停机。脚本会再次检查电机已停止和选中实例已冻结。成功导出后恢复运行并断开，不再复位，从而保留另一实例的记录。另一个正在记录的实例会在恢复后继续按业务状态工作；要保留两个现场，应在连接前分别冻结。历史 JSON 不修改。

## 验证范围

主机测试覆盖环形覆盖、序号/时间回绕、等待启动、异常冻结、独立重启、两个实例互不影响、不同数据类型适配、LQR 故障后 IMU 继续记录，以及导出格式。固件编译通过。本次未下载固件、未启动电机，尚未测量双实例板上轮询耗时。

## 一键导出与绘图

推荐在 VS Code 已停机、暂停的会话中执行 `trace-save`，无需逐项复制监视数据。终端独立采集、CSV 和 PNG 的使用方式见 [Trace 数据导出与绘图](Trace数据导出与绘图.md)。

## 2026-10-22 航向角速度反馈对照

独立 LqrTuning 新增 yaw_rate_scale，默认 0.5，仅缩放误差向量索引 4 的 psi_dot 反馈，不修改 K 或航向角项。pitch_rate_scale 仍为 0.5，试验时间仍为 7 秒。允许的 yaw_rate_scale 范围为 0.5–1.0，非 HOLD 状态可通过 LqrApp_SetTuning 修改，1.0 恢复原反馈。

本次对照使用 @lqrtrial；@lqrsoft 会额外把俯仰角速度系数改为 0.70，不适用于只改变航向反馈的比较。@lqrstatus 打印两个系数的千分数，默认均为 500。

每个 LQR 快照保存实际使用的两个系数，载荷格式升级为 3（72 字节），JSON schema_version 仍为 3。导出与绘图兼容旧 format=2（68 字节），旧格式的 yaw_rate_scale 按当时未缩放的含义记为 1.0。历史记录不改写。
