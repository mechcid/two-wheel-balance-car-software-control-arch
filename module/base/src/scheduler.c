#include "scheduler.h"
#include <limits.h>

//构建一个调度器，传入任务列表、任务数量和当前时间戳，初始化各任务的上次执行时间为当前时间戳
void Scheduler_Init(SchedulerTask *tasks, size_t count, uint32_t now_ms){
    if (!tasks) return;
    for (size_t i = 0; i < count; ++i) tasks[i].last_run_ms = now_ms;
}

//写好任务列表并初始化好后，放主循环里跑就是了
void Scheduler_Run(SchedulerTask *tasks, size_t count, uint32_t now_ms){
    if (!tasks) return;
    for (size_t i = 0; i < count; ++i) {
        //取出任务结构体列表中的任务地址，并校验是否合法
        SchedulerTask *task = &tasks[i];
        if (!task->enabled || !task->function || task->period_ms > INT32_MAX) continue;
        //校验周期是否到了，到了就执行任务函数，并更新上次执行时间。周期为零代表每轮都执行一次
        if ((uint32_t)(now_ms - task->last_run_ms) < task->period_ms) continue;
        task->last_run_ms = now_ms;
        task->function(task->context);
    }
}

//分别是在运行过程中试着动态修改任务的周期和启用状态，返回是否成功，暂时应该用不到
bool Scheduler_SetPeriod(SchedulerTask *task, uint32_t period_ms, uint32_t now_ms){
    if (!task || period_ms > INT32_MAX) return false;
    task->period_ms = period_ms;
    task->last_run_ms = now_ms;
    return true;
}

void Scheduler_SetEnabled(SchedulerTask *task, bool enabled, uint32_t now_ms){
    if (!task) return;
    if (task->enabled != enabled) task->last_run_ms = now_ms;
    task->enabled = enabled;
}
