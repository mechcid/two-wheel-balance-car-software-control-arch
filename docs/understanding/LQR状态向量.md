> 编辑时间：2026-11-02 20:44（北京时间，UTC+8）

# LQR 状态向量接口

本模块提供状态估计和只读接口，供 LQR 使用；控制量由 lqr_app 计算。单轮增量沿用当前代码中的 delta_displacement_m。

## 职责与调用

- Motor / motor_app：读取编码器、组合双轮反馈、执行现有占空比控制和故障停机。
- odometry_app：独立计算轮轴中点的平面 x/y 与沿车体前进方向的有符号速度。算法不依赖硬件，可直接做主机测试。
- status_state_app：读取姿态与同帧校准陀螺仪，检查时间与有效性，更新里程计并发布整份状态。

主循环先运行 SensorApp_Poll、MotionApp_Poll，等待新的姿态与 IMU 样本匹配，再依次执行 MotorApp_Update、StatusStateApp_Poll、LqrApp_Update。正常采样和控制沿用约 200 Hz 的输入节奏，不引入独立控制相位；没有新姿态时仍轮询状态与 LQR 的失效检查。

StatusStateApp_Init 在姿态和电机初始化之后注册双轮反馈回调。每帧反馈（包括停机补采样）都同步进入 StatusStateApp_Poll，由它检查新输入并调用内部 static Update。回调仅在调用期间借用反馈指针，返回前完成消费，不等待下一轮主循环，避免多次发布覆盖位移增量。主循环的 Poll 在无新输入时只检查失效。所有入口在主循环上下文运行，不支持中断或并发调用；不要覆盖反馈回调。

模块按四部分组织：

- `StatusState`：完整状态向量、测量时间戳、成功发布序号、有效性，以及 `motion`、`imu`、`motor` 三份来源标识，每份包含 `timestamp_us` 和 `sequence`。来源随成功输出一起发布，失败时保留旧来源。输出时间戳仍取电机采样时间。
- `StatusStateAppStatus`：未初始化、等待首帧、正常运行、不可用四种运行状态；当前原因位掩码 `faults`。不另设成功或失败计数，成功发布进度由样本序列号体现。
- 内部 `StatusStateApp_Update`：检查输入、计算、维护里程计，成功后一次性发布完整样本。失败时保留上次发布的数值、时间戳和序号，仅将样本标为不可用并记录原因。内部单独记录已消费电机序号，失败帧也不重复处理。
- 公开 `StatusStateApp_Poll`：按新反馈触发更新，并维护失效检查。重复帧不重复处理，轮询和超时检查不推进样本序列号；电机采样由主循环调度。

内部只保留三个含义不同的历史标志：`has_consumed_input` 表示处理过输入（包括失败帧），`has_published_sample` 表示成功发布过样本并已建立航向起点，`pitch_history_valid` 表示上一份俯仰仍可用于增量计算。初始化状态直接由 `status.state` 判断，不再重复维护布尔标志。样本是否可用于控制由 `valid` 表达，模块状态和原因供诊断查询。

StatusStateApp_GetState 和 StatusStateApp_GetStatus 分别返回只读数据与运行状态。StatusStateApp_GetVector(out) 按 [x,y,theta,psi,v,theta_dot,psi_dot] 复制 7 个 float，无效时返回 false 并保留 out 原值，调用者不能继续使用旧 out 控制。LQR 仍检查数据时效和对应的电机序号。

## 坐标与单位（接入 K 前必须核对）

x/y 单位 m，是轮轴中点在姿态固定参考系中的估计位置；车身 +X 必须对应两轮反馈正向，Z 向上，绕 +Z 的正转由 +X 转向 +Y。theta 使用当前姿态 pitch、绕 +Y，单位 rad；未增加安装偏角或符号修正。psi 使用 yaw 并连续展开，不在 ±π 处跳变，其零点沿用姿态启动航向。v 单位 m/s，为两轮 velocity_mps 加回车身俯仰分量后的平均值，每轮原始线速度由本次 RPM 与各自半径换算，不是非负速度模长，也不是瞬时端点速度。theta_dot、psi_dot 单位 rad/s，按 ZYX 欧拉角运动学由校准后的机体系陀螺仪求得：

```
theta_dot = gyro_y*cos(roll) - gyro_z*sin(roll)
psi_dot = (gyro_y*sin(roll) + gyro_z*cos(roll)) / cos(theta)
```

MotorState.velocity_mps 与 rpm 共用 feedback_valid。正常反馈和具有有效时间区间的启停补采样都更新这两个速度量；无有效区间或仅重建基准时，保留旧值但标记不可用。状态模块同时检查左右轮 feedback_valid，不从位移再次除以时间求速度。

此处 gyro 已换算为 rad/s，未使用互补滤波的重力修正项冒充真实角速度。|cos(theta)| <= 0.1 时拒绝欧拉角导数。未新增角速度或编码器速度滤波；实际噪声与控制频率须实机确认。

里程计在第一份可信采样的末端建立 x=y=0，不累计此前的运动。以后按左右增量均值 ds 和区间前后航向的最短角差中点计算 dx=ds*cos(mid)、dy=ds*sin(mid)。航向跨 ±π 时不会反向积分。普通启停不清零；StatusStateApp_Init 可在主循环、停机条件下显式重建位置原点，姿态本身的启动原点不被重置。

## 时间与有效性

双轮时间戳作为状态时间戳，保留原始姿态时间戳。姿态通过 `MotionSample.imu_source` 记录实际消费的 IMU。状态模块检查该来源的序号和时间戳与当前 IMU 都一致，并确认姿态时间戳与其输入时间相同；不匹配时拒绝组合，不缓存或找回旧 IMU。姿态与校准 IMU 必须对应同一读取时间，且姿态不得晚于双轮采样，最多允许落后 7500 us。这是有界延迟组合，不是硬件同步或插值；MPU 时间戳本身是读取开始时刻。中间航向计算使用两次通过检查的姿态，不能声称已经消除该延迟误差。

状态超过 20000 us 没成功更新则不可用，不会因为轮询而更新序号。初始化后尚无反馈时标记等待输入，样本无效。状态模块自身不改变电机输出；LQR 根据状态执行相应保护。正常周期约 200 Hz，停止补采样可能插入额外帧，不能将每次补采样都当作固定步长控制触发。

StatusStateApp_GetStatus()->faults 是当前位掩码（不是故障次数）：

| 位值 | 含义 |
|---|---|
| 0x01 | 双轮增量无效 |
| 0x02 | 姿态/IMU 无效、姿态不连续、数值异常或欧拉角奇异 |
| 0x04 | 姿态来源不匹配或时间偏差超限 |
| 0x08 | 位置尚未建立或累计不连续 |
| 0x10 | 已发布样本过期 |

重复电机序号不重复积分；已建立位置基准后，跳号或无效帧会保留已知位置并永久标记连续性丢失。内部里程计恢复时先重建航向区间起点，再累计后续可信小段；对外样本保持最后一次成功发布的数值，完整状态仍不可用，直到显式重建原点。姿态原有 discontinuity 也是持续标记，重置位置不会掩盖姿态历史缺样。valid 表示满足本接口的使用条件，不代表无航向漂移、无轮胎打滑或绝对定位准确。

## 查询与测试

串口发送 @state（实际 CRLF 结尾），查看样本时间戳、序号、valid、faults 和全部 7 个量；额外的 `STATE sources` 行按 motion、imu、motor 顺序显示每个来源的序号和时间戳。数值放大 1000 倍打印：位置 mm、角度 mrad、速度 mm/s、角速度 mrad/s。另提供 TelemetryApp_ReportState(StatusStateApp_GetState()) 周期打印，在 main.c 的 report_task 中注释或取消注释即可独立选择，与其他打印函数并列。受 @stream 总开关控制，无效帧也输出以便诊断。

周期格式：STATE,timestamp_us,sequence,valid,faults,x_mm,y_mm,theta_mrad,psi_mrad,v_mmps,theta_dot_mradps,psi_dot_mradps。faults 从模块运行状态读取，描述当前不可用原因；其为十六进制，其余整数为十进制；当前周期为 500 ms。

bash tests/status_state/run.sh 覆盖前后运动、两轮平均、重复/漏帧、序号及时间回绕、中间航向跨界、状态顺序和单位、欧拉角导数、过期/姿态失效与位置连续性。该测试模拟输入，不替代轮轴坐标方向与真实时间抖动的实机验证。


## 车身俯仰补偿

编码器测量轮子相对车身的转动。在当前约定（theta 为绕 +Y 的 pitch，车轮正转对应车体 +X 前进）下，omega_encoder = omega_absolute - theta_dot。因此在状态估计层逐轮修正：

```
v_absolute = velocity_mps + radius_m * theta_dot
delta_s_absolute = delta_displacement_m + radius_m * (theta_now - theta_previous)
```

底层 MotorState 保留相对车身的编码器反馈，不改变电机方向或标定。状态模块初始化改为 StatusStateApp_Init(left_radius_m, right_radius_m)，从现有 MotorConfig 复制轮半径，拒绝非有限或非正半径；不另写一套轮子参数。修正后的双轮行程与速度交给 odometry_app。

位移使用相邻有效姿态的角度差，不积分速度。首次样本只建立基准；无效/漏帧沿用原连续性失效规则，恢复时不跨缺失区间补算俯仰。该补偿沿用平面、小侧倾的平衡车模型，以及现有姿态与编码器最多 7.5 ms 的时间偏差；不是严格同时刻测量，动态变化时仍有残差。速度也仍含编码器区间平均值与姿态末端角速度之间的近似。

新增模拟测试：轮子对地静止时正/反俯仰，两轮半径不同，俯仰与真实平移同时发生，以及重复读取。静止轮测试应得到接近零的 x/y/v；实际安装符号还需固定轮子并俯仰车身验证，未声称已在实车消除残差。
