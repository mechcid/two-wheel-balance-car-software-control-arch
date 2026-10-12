#ifndef SCHEDULER_H
#define SCHEDULER_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*SchedulerFunction)(void *context);
typedef struct {
    SchedulerFunction function;
    void *context;
    uint32_t period_ms; /* 0 表示每轮执行一次；非零周期不得超过 INT32_MAX 毫秒。 */
    uint32_t last_run_ms;
    bool enabled;
} SchedulerTask;

/* 仅限单线程调用，不可重入；持续传入单调递增的毫秒计数。
 * 初始化保留各任务的周期和启用状态；非零周期任务首次执行前等待一个周期。
 * 每轮按数组顺序检查各任务一次，跳过已经错过的执行次数，不补执行。
 * 传入的时间是整轮共用的快照，不会在每个回调结束后自动更新。
 * 回调应及时返回；本调度器不提供实时操作系统的抢占能力。 */
void Scheduler_Init(SchedulerTask *tasks, size_t count, uint32_t now_ms);
void Scheduler_Run(SchedulerTask *tasks, size_t count, uint32_t now_ms);
bool Scheduler_SetPeriod(SchedulerTask *task, uint32_t period_ms, uint32_t now_ms);
void Scheduler_SetEnabled(SchedulerTask *task, bool enabled, uint32_t now_ms);
#endif
