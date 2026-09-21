# balance_car

STM32F103C8T6 + MPU6050 + SSD1306 OLED，使用硬件 I2C2，并通过 USART2/HC-05 输出采样数据。

## 使用

在 VSCode 中打开**本项目文件夹**：

- **Ctrl+Shift+B**：自动配置 → 编译 Debug → ST-Link 烧录 → 校验 → 复位运行。
- **F5**：自动编译 → 烧录并启动断点调试 → 停在 `main`；再按 F5 继续运行。
- 只编译、不连接硬件：菜单“终端 → 运行任务 → STM32: 编译 Debug”。

[VSCode 编译、烧录与调试说明](docs/VSCode编译烧录调试.md) · [前次整理与实测总结](docs/任务总结.md) · [当前调度器与采集架构](docs/调度器与采集架构.md)

## 目录

```text
Core/                  板级初始化、IRQ、main 入口（CubeMX）
Drivers/               STM32 HAL 与 CMSIS
module/inc/            UART、MPU6050、OLED、应用层头文件和字模
module/src/            对应实现；后续板级以上代码放这里
cmake/                 交叉编译工具链及 CubeMX 构建配置
.vscode/               图形化编译、烧录、调试和代码索引配置
docs/                  使用说明、必要硬件资料
build/Debug、Release/  自动生成的编译产物，可删除后重新构建
```

添加模块 `.c` 后，在顶层 `CMakeLists.txt` 的 `target_sources` 中登记；`module/inc` 已加入头文件搜索路径。

## 当前接口

| 设备 | 连接与配置 |
|---|---|
| MPU6050 | PB10=SCL、PB11=SDA，I2C2 100 kHz，实测地址 0x68 |
| SSD1306 128×64 OLED | 同一 I2C2，总线实测地址 0x3C（驱动也探测 0x3D） |
| HC-05 | USART2 PA2=TX、PA3=RX，9600 / 8N1，共地 |
| ST-Link | SWDIO、SWCLK、GND 与板卡相连，板卡正常供电 |

OLED 显示三轴加速度，`1000 mg = 1 g`；使用 ±2g 档，换算为 `raw × 1000 / 16384`，未添加零偏校准。

主循环通过独立 `scheduler` 调度模块运行各任务；采样周期、显示周期、上报周期均在 `Core/Src/main.c` 的任务表设置。`sensor_app` 私有保存原始数据，对外只提供修正后的 g、°/s、℃及时间戳。默认校准为零偏 0、比例 1。

串口每约 200 ms 输出 `MPU_C,毫秒,AX_mg,AY_mg,AZ_mg,TEMP_mC,GX_mdps,GY_mdps,GZ_mdps`，各物理量乘 1000 后以整数传输。此协议替代旧的原始 `MPU` 帧，旧解析程序需要更新。
命令须以 CRLF 结尾：`@ping`、`@status`、`@stream:0` / `@stream:1`。
`@status` 包含修正样本有效性、OLED、MPU 和 UART 状态。原始数据诊断不再作为普通交互命令；底层尾字节保护仍保留，原因见 [I²C 尾字节诊断](docs/i2c_tail_diagnostic.md)。

## CubeMX 再生成

本次对齐了 I2C2 的 `.ioc`、`i2c.c/.h`、`main.c` 调用与应用层句柄。
`.ioc` 还包含 TIM1/TIM4 编码器、TIM3 PWM 等配置，当前 Core 尚未完整生成这些功能，**当前固件没有实现电机控制**。
今后用 CubeMX 再生成时，确认 PB10/PB11=I2C2、USART2=PA2/PA3、USART2 中断和 9600 波特率，并检查 `main` 用户区的 模块初始化和 `Scheduler_Run`、UART 回调和 IRQ 是否保留。此次没有通过 CubeMX GUI 完整再生成工程。
