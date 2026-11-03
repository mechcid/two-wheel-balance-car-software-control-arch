#include "trajectory_app.h"
#include <math.h>

static TrajectorySample sample;
static TrajectoryStatus status;
static TrajectoryPoint hold;
static SampleSource origin_source;

bool TrajectoryApp_Init(void) {
    if (status.mode!=TRAJECTORY_NOT_INITIALIZED) return false;
    status.mode=TRAJECTORY_IDLE;
    return true;
}
static void TrajectoryApp_Update(uint32_t now) {
    TrajectorySample next={.data=hold,.timestamp_us=now,.sequence=sample.sequence+1,
        .valid=true,.valid_for_us=TRAJECTORY_VALID_FOR_US,.origin_source=origin_source};
    sample=next;
}
bool TrajectoryApp_Hold(const StatusState *origin,uint32_t now) {
    if (status.mode!=TRAJECTORY_IDLE || !origin || !origin->valid ||
        (uint32_t)(now-origin->timestamp_us)>TRAJECTORY_VALID_FOR_US ||
        !isfinite(origin->x_m) || !isfinite(origin->y_m) || !isfinite(origin->psi_rad)) return false;
    hold=(TrajectoryPoint){origin->x_m,origin->y_m,origin->psi_rad};
    origin_source=(SampleSource){origin->timestamp_us,origin->sequence};
    status.mode=TRAJECTORY_HOLD;
    TrajectoryApp_Poll(now);
    return true;
}
void TrajectoryApp_Stop(void) {
    if (status.mode==TRAJECTORY_NOT_INITIALIZED) return;
    status.mode=TRAJECTORY_IDLE;
    sample.valid=false;
}
void TrajectoryApp_Poll(uint32_t now) {
    if (status.mode!=TRAJECTORY_HOLD) return;
    if (!sample.valid || (uint32_t)(now-sample.timestamp_us)>=TRAJECTORY_UPDATE_PERIOD_US)
        TrajectoryApp_Update(now);
}
const TrajectorySample *TrajectoryApp_GetSample(void) { return &sample; }
const TrajectoryStatus *TrajectoryApp_GetStatus(void) { return &status; }
