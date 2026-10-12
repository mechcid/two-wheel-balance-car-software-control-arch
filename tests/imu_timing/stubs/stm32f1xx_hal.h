#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { uint32_t CYCCNT, CTRL; } TestDWT;
typedef struct { uint32_t DEMCR; } TestCoreDebug;
extern TestDWT test_dwt;
extern TestCoreDebug test_debug;
extern uint32_t test_mask;
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
static inline uint32_t HAL_RCC_GetHCLKFreq(void) { return 72000000U; }
static inline uint32_t __get_PRIMASK(void) { return test_mask; }
static inline void __disable_irq(void) { test_mask=1; }
static inline void __set_PRIMASK(uint32_t mask) { test_mask=mask; }
#endif
