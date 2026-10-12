#include "scheduler.h"
#include <limits.h>

void Scheduler_Init(SchedulerTask *tasks, size_t count, uint32_t now_ms)
{
    if (!tasks) return;
    for (size_t i = 0; i < count; ++i) tasks[i].last_run_ms = now_ms;
}
void Scheduler_Run(SchedulerTask *tasks, size_t count, uint32_t now_ms)
{
    if (!tasks) return;
    for (size_t i = 0; i < count; ++i) {
        SchedulerTask *task = &tasks[i];
        if (!task->enabled || !task->function || task->period_ms > INT32_MAX) continue;
        if ((uint32_t)(now_ms - task->last_run_ms) < task->period_ms) continue;
        task->last_run_ms = now_ms;
        task->function(task->context);
    }
}
bool Scheduler_SetPeriod(SchedulerTask *task, uint32_t period_ms, uint32_t now_ms)
{
    if (!task || period_ms > INT32_MAX) return false;
    task->period_ms = period_ms;
    task->last_run_ms = now_ms;
    return true;
}
void Scheduler_SetEnabled(SchedulerTask *task, bool enabled, uint32_t now_ms)
{
    if (!task) return;
    if (task->enabled != enabled) task->last_run_ms = now_ms;
    task->enabled = enabled;
}
