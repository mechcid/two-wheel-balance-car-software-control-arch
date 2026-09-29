# DAP 下载与调试验证

日期：2026-09-29。用户在 Windows 完成检查并调整操作后，Linux 实测成功；具体先前误操作内容未确认，不能据此认定为 Linux/OpenOCD 兼容性故障。

## 当前实测

- 设备：Horco CMSIS-DAP，VID:PID=faed:4873，序列号 6b12ae3c，固件 V208 v0.1。
- 电脑端 CONF.TXT 为 mode=master；本次通过无线主机连接 STM32。
- OpenOCD 0.12.0 自动选择 CMSIS-DAP v2 bulk 接口，SWD 100 kHz。
- 识别 SWD DPIDR=0x1ba01477、Cortex-M3 r2p0。
- 当前 Debug 固件构建通过，完整下载后返回 Verified OK，并成功复位。
- GDB 临时硬件断点命中 SensorApp_Update，调用栈可见 sample_task、Scheduler_Run；单步和变量读取成功。
- 暂停调试后曾出现 MPU 通信异常；再次复位后读取到 ready=true、reads=1020、failures=0、有效样本。没有在传感器事务期间暂停的实时性保证。
- 测试中读取两路 PWM 比较寄存器均为 0、PA15/STBY 输出为低；未发送电机启动指令。
- 测试结束已恢复程序运行并关闭 OpenOCD 服务。

## 复用命令

在项目根目录编译并下载：

```bash
cmake --build --preset Debug --parallel 2
openocd -f interface/cmsis-dap.cfg -c 'transport select swd' \
  -f target/stm32f1x.cfg -c 'adapter speed 100' \
  -c 'program build/Debug/balance_car.elf verify reset exit'
```

后续已将当前分支的 VSCode 默认下载和调试切换为 CMSIS-DAP：`tasks.json` 与 `launch.json` 共用 `.vscode/openocd.cfg`，SWD 100 kHz。Ctrl+Shift+B 编译、下载、校验并复位；F5 编译、下载并停在 main。原 ST-Link 配置由用户保留在主分支。未测试更高下载速度或无线距离。调试时应保持电机停止，CPU 暂停期间主循环中的软件超时保护无法执行。
