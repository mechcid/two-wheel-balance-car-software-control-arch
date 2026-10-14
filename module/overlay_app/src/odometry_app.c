#include "odometry_app.h"
#include <math.h>

//负责计算车体的位移和速度，使用双轮编码器的增量和姿态模块的航向角来计算车体在平面上的位置变化。

void OdometryApp_Init(OdometryState *s) {
    //本身不是什么传感器，只是一个集成的数据处理函数，初始化就直接清零
    *s=(OdometryState){0}; 
}

void OdometryApp_Update(OdometryState *s, float left, float right,
                        float left_velocity, float right_velocity, float psi,
                        uint32_t seq, uint32_t now, uint32_t dt, bool valid){
    //进行合法性校验
    if (s->initialized && seq==s->sequence) return;
    bool consecutive=!s->initialized || (uint32_t)(seq-s->sequence)==1;
    bool previous_valid=s->valid;

    //给计算数据打上序列号，时间戳，用于后续判断
    s->sequence=seq;
    s->timestamp_us=now;
    s->delta_x_m=s->delta_y_m=0;
    s->valid=false;
    valid=valid && dt>0 && dt<=50000 && isfinite(left) && isfinite(right) && isfinite(psi);
    if (!valid) {
        if (s->initialized) s->continuous=false;
        return;
    }

    //进行数据的处理计算
    float ds=0.5f*(left+right);
    float v=0.5f*left_velocity+0.5f*right_velocity;
    if (!isfinite(ds) || !isfinite(v)) {
        if (s->initialized) s->continuous=false;
        return;
    }
    s->v_mps=v;
    if (!s->initialized) {
        s->initialized=true;
        s->continuous=true;
    } else if (!previous_valid || !consecutive) {
        // 无可靠起点航向或漏帧：只重建基准，不能补算缺失路径。
        s->continuous=false;
    } else {
        float turn=atan2f(sinf(psi-s->last_psi_rad),cosf(psi-s->last_psi_rad));
        float mid=s->last_psi_rad+0.5f*turn;
        float dx=ds*cosf(mid), dy=ds*sinf(mid);
        if (!isfinite(s->x_m+dx) || !isfinite(s->y_m+dy)) {
            s->continuous=false;
            return;
        }
        s->delta_x_m=dx; s->delta_y_m=dy;
        s->x_m+=dx; s->y_m+=dy;
    }
    s->last_psi_rad=psi;
    s->valid=true;
}
