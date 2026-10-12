# balance_car

STM32F103C8T6 + MPU6050 + SSD1306 OLED，使用硬件 I2C2，并通过 USART2/HC-05 输出采样数据。

## 使用

在 VSCode 中打开**本项目文件夹**：

- **Ctrl+Shift+B**：自动配置 → 编译 Debug → CMSIS-DAP 烧录 → 校验 → 复位运行。
- **F5**：自动编译 → 烧录并启动断点调试 → 停在 `main`；再按 F5 继续运行。
- 只编译、不连接硬件：菜单“终端 → 运行任务 → STM32: 编译 Debug”。

[VSCode 编译、烧录与调试说明](docs/VSCode编译烧录调试.md) · [前次整理与实测总结](docs/任务总结.md) · [当前调度器与采集架构](docs/调度器与采集架构.md)

[MPU6050 仅姿态估计调整说明](docs/MPU6050仅姿态估计调整说明.md)：当前算法、接口、OLED 和串口协议，以及取消 IMU 位移积分的具体改动。

[本轮修改详解与代码阅读指南](docs/本轮修改详解与代码阅读指南.md) 保留前次实现的历史记录。当前 I2C2 保留 100 kHz；MPU 固定输出 200 Hz，DWT 提供统一微秒时间基准，TIM2 不再启动采集中断。详见 [200Hz IMU 与统一时间基准](docs/IMU_200Hz与统一时间基准.md)。实际板上采集吞吐尚未验证。

## 目录

```text
Core/                  板级初始化、IRQ、main 入口（CubeMX）
Drivers/               STM32 HAL 与 CMSIS
module/base/inc/       底层驱动与基础工具接口、OLED 字模
module/base/src/       硬件驱动、计时与通用调度实现
module/overlay_app/inc/ 上层算法与应用接口
module/overlay_app/src/ 姿态解算、传感器/电机/显示/遥测应用
cmake/                 交叉编译工具链及 CubeMX 构建配置
.vscode/               图形化编译、烧录、调试和代码索引配置
docs/                  使用说明、必要硬件资料
build/Debug、Release/  自动生成的编译产物，可删除后重新构建
```

添加模块 `.c` 后，在顶层 `CMakeLists.txt` 的 `target_sources` 中登记；`module/base/inc` 和 `module/overlay_app/inc` 均已加入头文件搜索路径，VSCode 的备用头文件路径也已同步。底层驱动及基础工具放在 `base`，PID、速度控制器、姿态算法和应用组合放在 `overlay_app`；两层内部各自区分 `inc` 与 `src`。

## 当前接口

| 设备 | 连接与配置 |
|---|---|
| MPU6050 | PB10=SCL、PB11=SDA，I2C2 100 kHz，实测地址 0x68 |
| SSD1306 128×64 OLED | 同一 I2C2，总线实测地址 0x3C（驱动也探测 0x3D） |
| HC-05 | USART2 PA2=TX、PA3=RX，9600 / 8N1，共地 |
| CMSIS-DAP | SWDIO、SWCLK、GND 与板卡相连，板卡正常供电 |

OLED 显示 Roll、Pitch、Yaw 三组姿态角（度，保留一位小数），发生采样中断时保留 GAP 提示；未取得有效、新鲜的解算数据时显示等待状态。传感器仍使用 ±2g 档，并使用 main 中配置的零偏及比例校准，三轴加速度输出取反。

主循环按 5 ms 固定截止时间优先采集并解算姿态；DWT 时间模块处理周期计数回绕，姿态 dt 来自相邻成功样本的微秒时间戳。其他任务仍由 `scheduler` 调度。OLED 每次发送前要求至少 2.5 ms 采集余量。`sensor_app` 发布 `ImuSample`：校准后的 g、°/s、微秒时间戳、软件发布序号和有效标志，20 ms 未更新视为过期。main 传入当前校准配置；传 NULL 才采用零偏 0、比例 1。

串口每约 200 ms 输出 `IMU,微秒,sequence,AX_mg,AY_mg,AZ_mg,GX_mdps,GY_mdps,GZ_mdps`，各物理量乘 1000 后以整数传输。温度列已删除，旧 `MPU_C` 解析程序需要更新。
命令须以 CRLF 结尾：`@ping`、`@status`、`@stream:0` / `@stream:1`。
命令数值使用轻量十进制解析，支持正负整数和普通小数，不支持科学计数法、空白或特殊浮点值；格式、精度和体积变化见 [UART 数值解析优化说明](docs/UART数值解析优化说明.md)。
`@status` 包含修正样本有效性、OLED、MPU 和 UART 状态。原始数据诊断不再作为普通交互命令；底层尾字节保护仍保留，原因见 [I²C 尾字节诊断](docs/i2c_tail_diagnostic.md)。

## 姿态估计

`motion_estimator` 仅融合加速度计和陀螺仪估计姿态，`motion_app` 负责采集与实际 dt，`timebase` 提供统一微秒时间，`TIM_IT` 保留为未启动的 TIM2 节拍工具。`@pose` 查看状态，`@stream:2` 输出 `POSE_US,时间_us,valid,roll_mdeg,pitch_mdeg,yaw_mdeg`。旧帧末尾的 X/Y/Z 位移字段和 `@pzero`、`@still` 命令已删除。

已取消 IMU 速度、位移积分。后续将用编码器实现小车位移估计，目前尚未加入编码器里程计算。详见 [当前实现说明](docs/MPU6050仅姿态估计调整说明.md)。

## 电机控制接入

已接入两轮电机应用、10 ms 调度与串口命令，驱动为 TB6612，每输出轴圈 1456 计数。
接线配置集中在 `Core/Inc/motor_board_config.h`；当前 `MOTOR_BOARD_READY=1`，上电默认停止。已实测确认 A=PB1/PWM2/编码器1，B=PB0/PWM1/编码器2；开环上限 ±40%，速度闭环尚未解锁。
详见 [电机控制接入说明](docs/电机控制接入说明.md)。默认停止、可选开环或 PID 速度闭环，尚未实现整车平衡。

## CubeMX 再生成

本次对齐了 I2C2 的 `.ioc`、`i2c.c/.h`、`main.c` 调用与应用层句柄。
`.ioc` 中 TIM1/TIM4 编码器、TIM3 PWM 尚未通过 CubeMX 完整生成；电机板级初始化暂由手写 `Core/Src/motor_board.c` 负责，当前已启用。再生成时需避免与 tim.c 重复初始化。
今后用 CubeMX 再生成时，确认 PB10/PB11=I2C2、USART2=PA2/PA3、USART2 中断和 9600 波特率，并检查 `main` 用户区的 模块初始化和 `Scheduler_Run`、UART 回调和 IRQ 是否保留。此次没有通过 CubeMX GUI 完整再生成工程。

默认下载与调试使用 `.vscode/openocd.cfg`：CMSIS-DAP、SWD、100 kHz。无线使用时电脑端为主机、板端接收器连接 SWD 并供电；有线使用时将设备切到有线模式。验证记录见 [DAP 下载调试验证](docs/DAP下载调试验证.md)。
