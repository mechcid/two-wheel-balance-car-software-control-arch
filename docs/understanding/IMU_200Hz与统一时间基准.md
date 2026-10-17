# 200Hz IMU 与统一时间基准

> 周期输出更新：三类独立打印函数现于 main 的 report_task 中组合，每 500 ms 输出；@stream 仅作为总开关，非零值不再选择数据种类。最新帧格式见 [README](../README.md#周期串口输出)。

2026-10-12。本说明替代旧文档中 TIM2 驱动采集、毫秒样本时间戳及温度遥测的描述。

## 数据接口

`ImuData` 保存校准后的 `accel[3]`（g）和 `gyro[3]`（°/s），轴顺序 X/Y/Z；`ImuSample` 包含 data、timestamp_us、sequence、valid。

时间戳在 DATA_RDY 检查完成后、数据突发读取前获取，读取成功才发布；它是主机读取开始时间，包含轮询相位误差，不能视为芯片内部采样时刻。sequence 从 1 开始，每份成功样本加一，uint32_t 自然回绕；重连保留序号，重新 SensorApp_Init 清零。序号不能计数芯片内部被覆盖的帧。

无新数据返回 HAL_BUSY，不增加序号或失败数。读取失败、重连、检测到 20 ms 未更新均清除 valid；旧数值和时间戳保留用于诊断。消费者仍需以无符号差值检查样本年龄。温度数据和温度校准已移除；驱动仍连续读取 15 字节，跳过温度解码，保留本板已有尾字节规避措施。

## MPU 与调度

初始化寄存器：PWR_MGMT_1=0x01、PWR_MGMT_2=0x00、CONFIG=0x03、SMPLRT_DIV=0x04、GYRO_CONFIG=0x00、ACCEL_CONFIG=0x00、INT_PIN_CFG=0x00、INT_ENABLE=0x01。输出固定 200 Hz，DLPF=3，±2g、±250°/s。重连恢复相同频率。

MotionApp 使用 5000 us 截止时间；按原计划推进，积压时只读取最新值，missed 统计错过的主机调度周期，不代表精确丢帧数。无新数据不更新姿态。dt 来自成功样本的时间戳差；超过 20 ms 则使姿态无效，下一帧按已有算法重新建立倾角，恢复时不积分缺失时段。

I2C 保持 100 kHz。状态检查和 15 字节读取约需 1.98 ms；OLED 单块定位加 16 字节数据约需 2.07 ms，均未计软件开销。OLED 发送前要求至少 2500 us 余量。这里是正常通信下的时间预算，并非硬实时保证；总线异常超时、设备重连仍可阻塞主循环。MCU 与 MPU 时钟独立，200 Hz 轮询也可能无新帧或漏帧；目前未启用 FIFO，也未接数据就绪中断。

## 时间基准

Timebase 在系统时钟配置后初始化一次。读取时累计 DWT 无符号周期差，保留不足一微秒的周期余数；用短临界区保护状态并恢复调用前 PRIMASK。重复初始化不复位计数器。TIM_IT 不再拥有或复位 DWT，MotionApp 不启动 TIM2 中断；TIM2 配置保留供后续精确定时任务使用。

72 MHz 下 DWT 每约 59.65 s 回绕，必须在此间隔内至少调用一次 Timebase_NowUs；主循环每轮调用满足正常运行要求。uint32_t 微秒时间戳约 71.6 分钟回绕，短期时间差以无符号减法计算。该时基不保证跨休眠、长时间调试暂停或动态变频连续。HAL/SysTick 仍供 HAL 超时、普通调度和现有电机模块使用；本次未修改电机时间接口。

## 遥测变更

- `@stream:1`：`IMU,timestamp_us,sequence,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps`。
- `@stream:2`：`POSE_US,timestamp_us,valid,roll_mdeg,pitch_mdeg,yaw_mdeg`。
- `@status`：SAMPLE 行包含 valid、us、seq。
- `@pose`：保留 MPU 配置频率、实际 dt、成功数、错过周期、无数据及失败数，移除 TIM 频率。

使用新帧前缀避免旧程序把微秒当成毫秒；电脑端旧解析程序需要适配。默认遥测周期仍为 200 ms，并非每帧发送。

## 验证

运行 `bash tests/imu_timing/run.sh`，使用主机 HAL/MPU 桩测试实际 timebase、sensor_app、motion_app 和姿态算法：DWT 回绕、微秒回绕、余数累积、中断屏蔽恢复、重复初始化、成功/无数据/超时/失败/重连、读取开始时间戳、跨回绕的 5 ms 调度、积压只读一次、缺样后的姿态恢复。

Debug 固件构建通过。主机测试不验证物理 I2C 时序；尚需板上验证开启 OLED 后 dt 分布、有效采样率、无数据计数和总线异常恢复。
