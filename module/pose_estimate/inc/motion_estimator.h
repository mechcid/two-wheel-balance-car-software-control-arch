#ifndef MOTION_ESTIMATOR_H
#define MOTION_ESTIMATOR_H
#include <stdbool.h>

/* 纯姿态算法，不依赖 HAL。机体和参考系均使用右手坐标系；参考系 Z 向上。
 * 四元数将机体坐标转换到参考坐标，顺序为 w、x、y、z。 */
typedef struct {
    float accel_sign;          /* 当前项目为 -1，把静止 Z=-1g 转成内部比力 Z=+1g。 */
    float correction_gain;     /* 重力方向互补反馈增益，单位 1/s。 */
    float accel_cutoff_hz;      /* 重力方向低通截止频率。 */
    float accel_tolerance_g;    /* 模长偏离 1g 的允许范围。 */
    float max_dt_s;            /* 超过此间隔时姿态失效，等待可信测量重建。 */
} MotionConfig;
typedef struct {
    float q[4];
    bool valid, accel_trusted;
    bool discontinuity;       /* 曾发生缺样，不能保证相对姿态的连续性；初始化清除。 */
} MotionState;
typedef struct {
    MotionConfig config;
    MotionState state;
    float gravity_lpf[3];
} MotionEstimator;

MotionConfig MotionEstimator_DefaultConfig(void);
bool MotionEstimator_Init(MotionEstimator *estimator, const MotionConfig *config);
/* accel 单位 g；gyro 单位度/秒；dt 是相邻成功采样的实际间隔，单位秒。
 * 首次样本用于建立倾角，启动时应保持静止；不计算平移速度或位移。
 * 拒绝异常输入并置 valid=false；恢复时重建倾角、保留上次航向，
 * 缺样期间的转动无法恢复，discontinuity 保留到下一次初始化。 */
bool MotionEstimator_Update(MotionEstimator *estimator, const float accel[3],
                            const float gyro[3], float dt_s);
/* 外部通信失败等缺样事件：保留四元数，置无效并标记姿态不连续。 */
void MotionEstimator_Invalidate(MotionEstimator *estimator);
/* 输出 roll、pitch、yaw，单位度；yaw 是相对启动方向，不能确定绝对航向。 */
void MotionEstimator_GetEuler(const MotionEstimator *estimator, float degrees[3]);
#endif
