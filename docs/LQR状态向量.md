# LQR 状态向量接口

本次提供状态估计和读取接口，不接入 K、不输出 LQR 控制量，也不修改电机接线、方向、比例和输出限制。单轮增量沿用当前代码中的 delta_displacement_m。

## 职责与调用

- Motor / motor_app：读取编码器、组合双轮反馈、执行现有占空比控制和故障停机。
- odometry_app：独立计算轮轴中点的平面 x/y 与沿车体前进方向的有符号速度。算法不依赖硬件，可直接做主机测试。
- status_state_app：读取姿态与同帧校准陀螺仪，检查时间与有效性，更新里程计并发布整份状态。

main 中 MotionApp_Poll 保持约 200 Hz；原 motor_task 替换为约 100 Hz 的 state_task，调用 StatusStateApp_Update，由它调用 MotorApp_Update。StatusStateApp_Init 在姿态和电机初始化之后注册双轮反馈回调；每帧反馈（包括停止补采样）都会在回调中更新状态。不要再额外调度 MotorApp_Update，也不要覆盖其反馈回调。主循环每轮调用 StatusStateApp_Poll，及时将过期状态置为无效。

StatusStateApp_GetState 返回带命名成员、时间戳、序号和错误位的最新状态。StatusStateApp_GetVector(out) 按 [x,y,theta,psi,v,theta_dot,psi_dot] 复制 7 个 float，无效时返回 false 并保留 out 原值，调用者不能继续使用旧 out 控制。控制器应按新 sequence 执行一次。当前没有控制器。

## 坐标与单位（接入 K 前必须核对）

x/y 单位 m，是轮轴中点在姿态固定参考系中的估计位置；车身 +X 必须对应两轮反馈正向，Z 向上，绕 +Z 的正转由 +X 转向 +Y。theta 使用当前姿态 pitch、绕 +Y，单位 rad；未增加安装偏角或符号修正。psi 使用 yaw 并连续展开，不在 ±π 处跳变，其零点沿用姿态启动航向。v 单位 m/s，为两轮 velocity_mps 的平均值，每轮由本次 RPM 与各自半径换算，不是非负速度模长，也不是瞬时端点速度。theta_dot、psi_dot 单位 rad/s，按 ZYX 欧拉角运动学由校准后的机体系陀螺仪求得：

```
theta_dot = gyro_y*cos(roll) - gyro_z*sin(roll)
psi_dot = (gyro_y*sin(roll) + gyro_z*cos(roll)) / cos(theta)
```

MotorState.velocity_mps 与 rpm 共用 feedback_valid。正常反馈和具有有效时间区间的启停补采样都更新这两个速度量；无有效区间或仅重建基准时，保留旧值但标记不可用。状态模块同时检查左右轮 feedback_valid，不从位移再次除以时间求速度。

此处 gyro 已换算为 rad/s，未使用互补滤波的重力修正项冒充真实角速度。|cos(theta)| <= 0.1 时拒绝欧拉角导数。未新增角速度或编码器速度滤波；实际噪声与控制频率须实机确认。

里程计在第一份可信采样的末端建立 x=y=0，不累计此前的运动。以后按左右增量均值 ds 和区间前后航向的最短角差中点计算 dx=ds*cos(mid)、dy=ds*sin(mid)。航向跨 ±π 时不会反向积分。普通启停不清零；StatusStateApp_Init 可在主循环、停机条件下显式重建位置原点，姿态本身的启动原点不被重置。

## 时间与有效性

双轮时间戳作为状态时间戳，保留原始姿态时间戳。姿态与校准 IMU 必须对应同一读取时间，且姿态不得晚于双轮采样，最多允许落后 7500 us。这是有界延迟组合，不是硬件同步或插值；MPU 时间戳本身是读取开始时刻。中间航向计算使用两次通过检查的姿态，不能声称已经消除该延迟误差。

状态超过 20000 us 没更新则不可用，不会因为轮询而更新序号。有效性检查不改变电机输出，尚未接入控制层安全决策。周期约 100 Hz，停止补采样可能插入额外帧；后续离散 LQR 的设计周期需匹配，不能直接把每次补采样当作固定步长控制触发。

faults 是当前位掩码（不是故障次数）：

| 位值 | 含义 |
|---|---|
| 0x01 | 双轮增量无效 |
| 0x02 | 姿态/IMU 无效、姿态不连续、数值异常或欧拉角奇异 |
| 0x04 | 姿态来源不匹配或时间偏差超限 |
| 0x08 | 位置尚未建立或累计不连续 |
| 0x10 | 未收到状态或状态过期 |

重复电机序号不重复积分；跳号或无效帧会保留已知位置并永久标记连续性丢失。恢复时先重建航向区间起点，再累计后续可信小段，完整状态仍保持无效，直到显式重建原点。姿态原有 discontinuity 也是持续标记，重置位置不会掩盖姿态历史缺样。valid 表示满足本接口的使用条件，不代表无航向漂移、无轮胎打滑或绝对定位准确。

## 查询与测试

串口发送 @state（实际 CRLF 结尾），查看两种时间戳、序号、valid、faults 和全部 7 个量。数值放大 1000 倍打印：位置 mm、角度 mrad、速度 mm/s、角速度 mrad/s。另提供 TelemetryApp_ReportState(StatusStateApp_GetState()) 周期打印，在 main.c 的 report_task 中注释或取消注释即可独立选择，与其他打印函数并列。受 @stream 总开关控制，无效帧也输出以便诊断。

周期格式：STATE,timestamp_us,sequence,valid,faults,x_mm,y_mm,theta_mrad,psi_mrad,v_mmps,theta_dot_mradps,psi_dot_mradps。faults 为十六进制，其余整数为十进制；当前周期为 500 ms。

bash tests/status_state/run.sh 覆盖前后运动、两轮平均、重复/漏帧、序号及时间回绕、中间航向跨界、状态顺序和单位、欧拉角导数、过期/姿态失效与位置连续性。该测试模拟输入，不替代轮轴坐标方向与真实时间抖动的实机验证。
