#ifndef TRACE_ADAPTERS_H
#define TRACE_ADAPTERS_H
#include "trace_sources.h"
void LqrTrace_Prepare(void *context);
void ImuTrace_Prepare(void *context);
TraceSourceStatus LqrTrace_Check(void *context);
TraceSourceStatus ImuTrace_Check(void *context);
#endif
