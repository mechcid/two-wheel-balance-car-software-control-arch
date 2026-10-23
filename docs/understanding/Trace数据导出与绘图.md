# Trace 数据导出与绘图

用途：把 MCU RAM 中已经冻结的环形记录按时间顺序读出，保存为 JSON，再生成 CSV、PNG 和摘要。脚本不烧录、不复位、不启动电机，也不会重新 Arm 记录器。

## 推荐：在当前 VS Code 调试会话里导出

1. 继续运行程序，扶护小车，发送串口命令 `@lqrtrial`。当前试验上限是 7 秒，发生故障可能提前停机。
2. 确认电机已经停下，再点击暂停。不要在电机运行时暂停 CPU；PWM 外设可能继续输出，软件保护此时不能执行。
3. 在 VS Code 的**调试控制台**（不是终端、串口或监视输入框）输入：

   ```text
   trace-save
   ```

`.vscode/launch.json` 已加入注册此命令的启动配置，下次启动调试自动加载。若当前调试会话是在修改配置之前启动的，不用重启，在调试控制台先执行一次：

```text
source /home/mechcid/work/balance_car/tools/trace_gdb.py
trace-save
```

命令先检查目标暂停、LQR 未运行、STBY/PWM 已关闭、记录器已冻结，然后导出 LQR。完成后保持当前暂停状态，不接管或断开 VS Code 的调试器。

IMU 使用 `trace-save imu`。IMU 可通过 `@imutracearm` 开始记录、`@imutracefreeze` 冻结；即使只导出 IMU，也要先停止电机再暂停。必要时可以指定新目录：`trace-save lqr /tmp/my-lqr-record`。

## 没有调试会话时：终端脚本

前提：电机已停止，Trace 已冻结，MCU 没有复位或断电，并且 ELF 对应板上当前固件。

在项目根目录运行：

```bash
python3 tools/capture_trace.py
```

该脚本自动启动临时 OpenOCD、连接 GDB、比较 ELF 与目标的 `.text` 段、读取冻结记录，再恢复 CPU、断开并关闭自己启动的 OpenOCD。最后离线生成 CSV 和图。它不执行烧录或复位。

如果 VS Code 已有调试会话/OpenOCD，脚本会拒绝启动第二个调试器，请直接使用上面的 `trace-save`。不要为了运行终端脚本而随意结束一个可能重置目标的调试会话，否则 RAM 数据可能丢失。

可选参数：

```bash
python3 tools/capture_trace.py --source imu
python3 tools/capture_trace.py --elf build/Debug/balance_car.elf --output-dir /tmp/trace-run-01
```

--output-dir 必须是尚不存在的新目录。默认 GDB 超时 90 秒，可用 --timeout 调整。连接异常时会尝试恢复目标并关闭自己的服务；若恢复失败会明确提示，不能把通信故障下的恢复当成保证。

## 输出文件

默认写入：

```text
docs/experiments/traces/<UTC日期时间>_lqr/
    lqr_trace.json
    lqr_trace.csv
    lqr_trace.png
    lqr_trace_summary.json
```

终端脚本还保存 openocd.log 和 gdb.log。每次目录名含微秒时间；文件拒绝覆盖。

- JSON：原始帧、状态、来源/格式版本、采集时间、ELF 路径及 SHA-256，适合保留和交给分析工具。
- CSV：每行一条记录，字段名含单位，方便表格软件和 Python 读取。停止/故障等事件保留，但测量列为空，不补零或沿用旧测量。
- PNG：俯仰、角速度、位置/速度、左右占空比、请求力矩、轮速、航向和时间间隔/输入年龄。红色竖线表示终止事件。
- 摘要：有效记录数量、保留窗口长度、事件原因、样本间隔、俯仰范围及峰值占空比等；不自动推断故障根因。

**分析时发整个目录，或至少发 JSON 即可。**无需从监视窗口逐条复制 64 条数据。

## 离线重新分析

此命令完全不连接硬件：

```bash
python3 tools/trace_report.py docs/experiments/traces/<目录>/lqr_trace.json --output-dir /tmp/trace-analysis-new
```

仅导出 CSV 和摘要（不需要 matplotlib）：

```bash
python3 tools/trace_report.py <记录.json> --output-dir /tmp/trace-csv-new --no-plot
```

绘图依赖 Python 3 和 matplotlib。当前环境已具备依赖。GDB 命令默认使用 /usr/bin/python3，也可在启动 VS Code/GDB 前通过 TRACE_PYTHON 指定绘图解释器。绘图失败时原始 JSON 仍保留，可修复依赖后离线重试。

## 如何解释数据

LQR 记录器只有 64 槽，200 Hz 下保留约 0.32 秒，终止事件也占一槽。7 秒试验结束后的文件是最后一段窗口，并非整个试验。横轴零点是最旧的保留帧，不是试验启动时刻。

脚本先还原环形数组的时间顺序，再处理 uint32 微秒时间戳回绕。有效测量与事件分开处理，事件不参与曲线计算。只有终止事件而没有正常数据时，也可导出；这往往表示业务在第一帧有效输出前就退出，不能据此分析闭环振荡。

该工具不会自动调整 K、控制限制或调试系数。先保留原始数据，再结合实车现象分析。

## 已验证范围

运行 `bash tests/trace_tools/run.sh`，验证时间戳回绕、CSV 空事件列、PNG 生成、环形读取顺序、冻结/停机检查和拒绝覆盖。测试使用明确生成的模拟数据，不代表实车测量。命令注册还通过了本地 GDB 加载检查。尚未执行本次新工具的板上导出。

当前航向反馈对照固件的 LQR 载荷格式为 format_id=3，额外记录 yaw_rate_scale。基准历史 format_id=2 对应 yaw_rate_scale=1.0，工具同时支持两种格式。新试验正常默认记录 pitch_rate_scale=0.5、yaw_rate_scale=0.5。若已有调试会话仍加载旧版 trace-save，请重新 source tools/trace_gdb.py；新启动会话会自动加载更新版本。
