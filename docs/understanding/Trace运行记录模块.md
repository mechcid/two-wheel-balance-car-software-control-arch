> 编辑时间：2026-11-03 19:11（北京时间，UTC+8）

# Trace 接入、操作与验证

设计目的、职责划分、状态与定频记录约定统一见 [追踪模块架构](../arch/追踪模块.md)。本文只保留当前项目的接入方式、配置位置和操作说明。

## 初始化与调用

```c
static LqrTrace lqr_trace;
static ImuTrace imu_trace;

if (!LqrTrace_Init(&lqr_trace) || !ImuTrace_Init(&imu_trace)) Error_Handler();
if (!DebugApp_Init(&lqr_trace, &imu_trace)) Error_Handler();
```

主循环由 `DebugApp_Poll(now)` 统一推进两个记录对象，不必额外重复轮询。独立使用当前适配对象时，通过管理入口先准备源变量：

```c
TraceApp_Arm(&lqr_trace.recorder);
TraceSources_Poll(&lqr_trace.recorder, now_us);
TraceSources_Freeze(&lqr_trace.recorder, now_us);

LqrTraceFrame frame;
if (LqrTrace_Read(&lqr_trace, 0, &frame)) {
    // 冻结后读取最旧的一条保留记录。
}
```

## 配置位置与当前容量

频率在 `module/debug/src/trace_sources.c` 的 `registrations` 表中设置，当前 LQR 与 IMU 均为 200 Hz。数组容量在 `module/debug/inc/trace_sources.h` 中分别配置。

| 实例 | format_id | 载荷字节 | 帧字节 | 容量 | 数组字节 |
| --- | ---: | ---: | ---: | ---: | ---: |
| LQR | 4 | 84 | 108 | 64 | 6912 |
| IMU | 2 | 36 | 60 | 24 | 1440 |

两个数组合计 8352 字节，另有源变量及管理对象。200 Hz 下分别保留约 0.32 秒、0.12 秒，终止快照也占一槽。调整容量后重新检查链接 RAM 占用；链接值不等于运行时栈峰值。

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

导出脚本仍为 tools/export_lqr_trace.gdb，通过环境变量 TRACE_SOURCE=lqr（默认）或 imu 选择实例。LQR_TRACE_OUT 兼容原路径覆盖变量；默认输出 docs/experiments/lqr_trace.json 或 imu_trace.json。输出保持 schema_version=3。新格式导出终止快照，旧格式事件仍无载荷；拒绝覆盖已有文件。

必须在连接会暂停 CPU 的调试器之前停止电机；冻结 Trace 不等于停机。脚本会再次检查电机已停止和选中实例已冻结。成功导出后恢复运行并断开，不再复位，从而保留另一实例的记录。另一个正在记录的实例会在恢复后继续按业务状态工作；要保留两个现场，应在连接前分别冻结。历史 JSON 不修改。

## 验证与导出

```bash
bash tests/trace/run.sh
bash tests/trace_tools/run.sh
bash tests/lqr/run.sh
bash tests/control_pipeline/run.sh
cmake --build --preset Debug
```

测试覆盖定频复制、时间回绕、错过周期不补写、故障立即冻结、终止载荷、独立环形实例、业务适配，以及新旧导出格式。当前实现通过主机测试和构建，尚未完成此次改造的板上导出与耗时验证。

推荐在 VS Code 已停机、暂停的会话中执行 `trace-save`。完整操作见 [Trace 数据导出与绘图](Trace数据导出与绘图.md)。

## 2026-10-22 航向角速度反馈对照

独立 LqrTuning 新增 yaw_rate_scale，默认 0.5，仅缩放误差向量索引 4 的 psi_dot 反馈，不修改 K 或航向角项。pitch_rate_scale 仍为 0.5，试验时间仍为 7 秒。允许的 yaw_rate_scale 范围为 0.5–1.0，非运行状态可通过 LqrApp_SetTuning 修改，1.0 恢复原反馈。

本次对照使用 @lqrtrial；@lqrsoft 会额外把俯仰角速度系数改为 0.70，不适用于只改变航向反馈的比较。@lqrstatus 打印两个系数的千分数，默认均为 500。

每个 LQR 快照保存实际使用的两个系数，当时载荷格式升级为 3（72 字节），JSON schema_version 仍为 3。导出与绘图兼容旧 format=2（68 字节），旧格式的 yaw_rate_scale 按当时未缩放的含义记为 1.0。历史记录不改写。
