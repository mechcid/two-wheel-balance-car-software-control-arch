# balance_car

这是一个基于 LQR 控制的双轮自平衡小车项目，主控为 STM32F103C8T6，搭配 MPU6050、SSD1306 OLED 和 TB6612 电机驱动。项目使用硬件 I2C2 连接传感器与显示屏，通过 USART2 / HC-05 进行串口调试和数据输出。

## 项目进展与个人回顾

从 2026 年 8 月 20 日开始，到 10 月 23 日，断断续续做了两个多月。目前，小车已经能够在原点附近保持直立，但仍有小幅振荡，距离我希望达到的稳定效果还有差距。我对这个结果还不满意，只是进一步定位和解决问题需要投入更多时间。之后我希望继续跟进这个项目，更新进度则取决于时间和精力。

开发过程中，我使用了 AI 辅助编写和调整代码。回头看，有些功能的扩展已经超出了我当时能够充分理解和掌握的范围。例如，用于记录运行数据的 Trace 模块确实帮助了调试，但我目前还不能完整解释它的内部实现和设计取舍。这是这个项目让我感到遗憾的地方，也是之后需要补上的一课：除了让小车运行起来，我还希望真正理解自己项目中的代码。

当前版本仍处于实验和调试阶段。已经接入姿态估计、编码器里程计、原点直立控制以及运行数据记录；轨迹跟踪等运动控制尚未实现。

LQR 使用 5 ms 控制周期，按 12 V 名义供电进行力矩到占空比的换算，上电默认停用。详见 [LQR 原点直立接入](docs/LQR原点直立接入.md)。部分历史文档仍保留早期实现和验证状态，阅读时请结合当前代码区分。

## 使用

在 VS Code 中打开**本项目文件夹**：

- **Ctrl+Shift+B**：自动配置 → 编译 Debug → CMSIS-DAP 烧录 → 校验 → 复位运行。
- **F5**：自动编译 → 烧录并启动断点调试 → 停在 `main`；再按 F5 继续运行。
- 只编译、不连接硬件：菜单“终端 → 运行任务 → STM32: 编译 Debug”。

[VS Code 编译、烧录与调试说明](docs/understanding/VSCode编译烧录调试.md) · [前次整理与实测总结](docs/understanding/任务总结.md) · [当前调度器与采集架构](docs/understanding/调度器与采集架构.md)

[MPU6050 仅姿态估计调整说明](docs/understanding/MPU6050仅姿态估计调整说明.md)：当前算法、接口、OLED 和串口协议，以及取消 IMU 位移积分的具体改动。

[本轮修改详解与代码阅读指南](docs/understanding/本轮修改详解与代码阅读指南.md) 保留前次实现的历史记录。当前 I2C2 保留 100 kHz；MPU 固定输出 200 Hz，DWT 提供统一微秒时间基准，TIM2 不再启动采集中断。详见 [200Hz IMU 与统一时间基准](docs/understanding/IMU_200Hz与统一时间基准.md)。已完成部分板上采集与显示验证；闭环运行效果仍在持续调试。

## 目录

```text
Core/                  板级初始化、IRQ、main 入口（CubeMX）
Drivers/               STM32 HAL 与 CMSIS
module/base/inc/       底层驱动与基础工具接口、OLED 字模
module/base/src/       硬件驱动、计时与通用调度实现
module/overlay_app/inc/ 上层算法与应用接口
module/overlay_app/src/ 姿态解算、传感器/电机/显示/遥测应用
module/deferred/       暂存算法，不参与当前构建
cmake/                 交叉编译工具链及 CubeMX 构建配置
.vscode/               图形化编译、烧录、调试和代码索引配置
docs/                  使用说明、硬件资料、控制参数与试验记录
tools/                 参数生成、Trace 导出与离线绘图脚本
tests/                 主机侧模块测试
build/Debug、Release/  自动生成的编译产物，可删除后重新构建
```

添加模块源文件后，在顶层 `CMakeLists.txt` 的 `target_sources` 中登记；`module/base/inc` 和 `module/overlay_app/inc` 均已加入头文件搜索路径，VS Code 的备用头文件路径也已同步。底层驱动及基础工具放在 `base`，姿态算法和应用组合放在 `overlay_app`；PID 和速度控制器暂存于 `deferred/speed_control`；两层内部各自区分 `inc` 与 `src`。

## 当前接口

| 设备 | 连接与配置 |
|---|---|
| MPU6050 | PB10=SCL、PB11=SDA，I2C2 100 kHz，实测地址 0x68 |
| SSD1306 128×64 OLED | 同一 I2C2，总线实测地址 0x3C（驱动也探测 0x3D） |
| HC-05 | USART2 PA2=TX、PA3=RX，9600 / 8N1，共地 |
| CMSIS-DAP | SWDIO、SWCLK、GND 与板卡相连，板卡正常供电 |

OLED 显示 Roll、Pitch、Yaw 三组姿态角（度，保留一位小数），发生采样中断时保留 GAP 提示；未取得有效、新鲜的解算数据时显示等待状态。传感器仍使用 ±2g 档，并使用 main 中配置的零偏及比例校准，三轴加速度输出取反。

主循环按 5 ms 固定截止时间优先采集并解算姿态；DWT 时间模块处理周期计数回绕，姿态 dt 来自相邻成功样本的微秒时间戳。其他任务仍由 `scheduler` 调度。OLED 采用 8 字节分块发送，每次发送前要求至少 1.7 ms 采集余量。`sensor_app` 发布 `ImuSample`：校准后的 g、°/s、微秒时间戳、软件发布序号和有效标志，20 ms 未更新视为过期。main 传入当前校准配置；传 NULL 才采用零偏 0、比例 1。

启用 IMU 周期输出后，串口发送 `IMU,微秒,sequence,AX_mg,AY_mg,AZ_mg,GX_mdps,GY_mdps,GZ_mdps`，各物理量乘 1000 后以整数传输。温度列已删除，旧 `MPU_C` 解析程序需要更新。
命令须以 CRLF 结尾：`@ping`、`@status`、`@stream:0` / `@stream:1`。
命令数值使用轻量十进制解析，支持正负整数和普通小数，不支持科学计数法、空白或特殊浮点值；格式、精度和体积变化见 [UART 数值解析优化说明](docs/understanding/UART数值解析优化说明.md)。
`@status` 包含修正样本有效性、OLED、MPU 和 UART 状态。原始数据诊断不再作为普通交互命令；底层尾字节保护仍保留，原因见 [I²C 尾字节诊断](docs/understanding/i2c_tail_diagnostic.md)。

## 姿态估计

`motion_estimator` 仅融合加速度计和陀螺仪估计姿态，`motion_app` 负责采集与实际 dt，`timebase` 提供统一微秒时间，`TIM_IT` 保留为未启动的 TIM2 节拍工具。`@pose` 查看状态，独立的 `TelemetryApp_ReportAttitude()` 输出 `POSE_US,时间_us,valid,roll_mdeg,pitch_mdeg,yaw_mdeg`。旧帧末尾的 X/Y/Z 位移字段和 `@pzero`、`@still` 命令已删除。

已取消单纯依靠 IMU 加速度积分估计速度和位移的实现。当前通过编码器估计小车位移与速度，并在整车状态模块中结合车身俯仰进行修正。姿态模块的调整背景见 [当前实现说明](docs/understanding/MPU6050仅姿态估计调整说明.md)。

## 电机控制接入

已接入双轮电机控制、编码器反馈和串口命令。驱动为 TB6612，编码器每个输出轴转动一圈对应 1456 个计数；当前电机反馈与整车状态按 5 ms 周期更新。
接线配置集中在 `Core/Inc/motor_board_config.h`；当前 `MOTOR_BOARD_READY=1`，上电默认停止。已实测确认 A=PB1/PWM2/编码器1，B=PB0/PWM1/编码器2；开环上限 ±40%，PID 和速度控制器已移到暂存目录，不参与当前运行。
开环使用方式见 [电机开环与测速](docs/understanding/电机开环与测速.md)。电机应用接收占空比目标并报告实测 RPM；LQR 在此基础上计算闭环控制输出。旧命令 `@mrun` 已移除，测速和保护使用 DWT 微秒时间。[电机控制接入说明](docs/understanding/电机控制接入说明.md) 保留早期接入及实测记录。

原点直立调试可使用 `@lqrtrial`，当前试验时长为 7 秒，结束后自动停机；发生故障时可能提前退出。`@lqrstatus` 查询控制状态，`@mstop` 用于停止电机并退出闭环。暂停 CPU 前应先确认电机已停止，因为暂停软件执行不一定会停止 PWM 输出。

## CubeMX 再生成

I2C2 配置已在 `.ioc`、`i2c.c/.h`、`main.c` 调用和应用层句柄之间保持一致。
TIM1/TIM4 编码器、TIM3 PWM 参数统一在 `.ioc` 和 `tim.c`：PWM PSC=0、ARR=3599，编码器滤波=6、输入上拉。`motor_board.c` 仅检查配置并启动 HAL，不复位或重写定时器。再生成需保留 TIM3 用户区关闭 CCR 预装载的代码。
今后用 CubeMX 再生成时，确认 PB10/PB11=I2C2、USART2=PA2/PA3、USART2 中断和 9600 波特率，并检查 `main` 用户区的模块初始化、控制循环和 `Scheduler_Run`、UART 回调和 IRQ 是否保留。当前修改尚未经过 CubeMX GUI 完整再生成验证。

默认下载与调试使用 `.vscode/openocd.cfg`：CMSIS-DAP、SWD、100 kHz。无线使用时电脑端为主机、板端接收器连接 SWD 并供电；有线使用时将设备切到有线模式。验证记录见 [DAP 下载调试验证](docs/understanding/DAP下载调试验证.md)。

## 周期串口输出

`main.c` 的 `report_task()` 集中管理周期输出。当前启用的是整车状态输出 `TelemetryApp_ReportState()`；IMU、姿态和电机输出调用暂时注释，可按调试需要启用。周期报告间隔为 500 ms，启用多类输出时需注意 9600 / 8N1 串口的带宽和发送队列容量。

`@stream:0` 暂停全部周期输出，`@stream:1` 恢复；其他非零值也表示恢复，`@stream:2` 不再单独选择姿态。`@pose`、`@mstatus`、`@status` 等手动查询不受总开关限制。

以下是可选 IMU、姿态和电机输出的帧格式：

- `IMU,timestamp_us,sequence,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps`：只输出有效且新鲜样本。
- `POSE_US,timestamp_us,valid,roll_mdeg,pitch_mdeg,yaw_mdeg`：失效或过期时输出 valid=0，角度为保留值。
- `MOTOR,timestamp_us,mode,left_mrpm,right_mrpm,left_mm,right_mm,left_speed_valid,right_speed_valid,left_distance_valid,right_distance_valid`：停止和故障状态也输出，数值是否可用由标志指明。mode=0/1/2/3 分别为 STOPPED/DUTY/FAULT/UNCONFIGURED。

以上物理量以整数发送（mrpm 为 RPM × 1000，mm 为 m × 1000）。若同时启用这三类输出，按 32 位最宽数值估算，每轮约 233 字节，500 ms 周期约占用 466 字节/秒。9600 / 8N1 的理论数据吞吐量约为 960 字节/秒，其他周期输出和手动查询仍会占用剩余带宽；可通过 `@status` 查看 `tx_drop`，判断是否发生发送丢弃。

## 运行数据记录与分析

[Trace 运行记录模块](docs/understanding/Trace运行记录模块.md)：LQR/IMU 类型化独立实例、环形缓冲区、业务状态门控和冻结后导出。

[Trace 数据导出与绘图](docs/understanding/Trace数据导出与绘图.md)：在 VS Code 调试控制台运行 `trace-save`，自动保存 JSON、CSV、曲线和摘要；也支持终端脚本导出。
