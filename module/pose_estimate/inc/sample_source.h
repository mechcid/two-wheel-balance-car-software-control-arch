#ifndef SAMPLE_SOURCE_H
#define SAMPLE_SOURCE_H
#include <stdint.h>

/* 样本来源标识：时间戳使用共同的 uint32_t 微秒基准，序号属于来源模块。
 * 两者均允许回绕；这是来源记录，不代表该来源当前仍有效。
 * 描述当前初始化生命周期内的样本，不跨模块重置比较。 */
typedef struct {
    uint32_t timestamp_us;
    uint32_t sequence;
} SampleSource;
#endif
