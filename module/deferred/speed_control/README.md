# 暂存的速度闭环

当前固件只使用开环占空比输出和编码器测速，此目录不加入 CMake 源文件或头文件搜索路径。

- `pid.hpp`：原位置式/增量式 PID，内容保留。
- `motor_speed_controller.hpp`：原单轮速度闭环组合，内容保留，依赖 base 的 Motor。

原配置记录：左右 Kp/Ki/Kd 均为 0，位置式 PID，速度闭环允许开关为 false，RPM 目标上限为 120（软件暂定值，非额定转速）。这些参数已从有效板级配置移除。

原接入方式：MotorApp 为每轮构造一个 MotorSpeedController；SetRPM 写目标并启用控制器；更新时执行测速、PID、占空比输出；启停重置 PID。以后恢复需要重新设计应用接口、参数和测试，并适配统一的微秒时间。当前没有 MOTOR_SPEED 状态、MotorApp_SetRPM 接口或 @mrun 命令。
