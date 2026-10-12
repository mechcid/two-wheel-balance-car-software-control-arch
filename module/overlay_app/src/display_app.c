#include "display_app.h"
#include "oled.h"
#include <math.h>
#include <stdio.h>

/* OLED 每行最多 16 个字符，标题加三个姿态角；每次完整覆盖旧行。 */
static void show_waiting(void)
{
    OLED_ShowString(1, 1, "POSE waiting... ");
    OLED_ShowString(2, 1, "Roll      --    ");
    OLED_ShowString(3, 1, "Pitch     --    ");
    OLED_ShowString(4, 1, "Yaw       --    ");
}

HAL_StatusTypeDef DisplayApp_Init(I2C_HandleTypeDef *bus)
{
    HAL_StatusTypeDef result = OLED_Init(bus);
    if (result == HAL_OK) show_waiting();
    return result;
}

/* 手工拆分整数和小数，避免启用体积较大的浮点 printf。
 * 角度保留一位小数；舍入后的数值超出栏宽时显示 OVF。 */
static void format_angle(float degrees, char text[8])
{
    if (!isfinite(degrees)) {
        snprintf(text, 8, "--");
        return;
    }
    float magnitude = fabsf(degrees) * 10.0f + 0.5f;
    if (magnitude >= 10000.0f) {
        snprintf(text, 8, "OVF");
        return;
    }
    unsigned tenths = (unsigned)magnitude;
    if (tenths > 9999) {
        snprintf(text, 8, "OVF");
        return;
    }
    /* 接近零时显示 +0.0，避免出现 -0.0。 */
    snprintf(text, 8, "%c%u.%u", degrees < 0 && tenths ? '-' : '+',
             tenths / 10, tenths % 10);
}

void DisplayApp_Render(const MotionState *motion, const float degrees[3], bool available)
{
    if (!OLED_IsReady()) return;
    if (!motion || !degrees || !motion->valid || !available) {
        show_waiting();
        return;
    }
    OLED_ShowString(1, 1, motion->discontinuity ? "ATTITUDE GAP    " : "ATTITUDE / deg  ");
    static const char *labels[] = {"Roll", "Pitch", "Yaw"};
    for (unsigned i = 0; i < 3; ++i) {
        char angle[8], row[17];
        format_angle(degrees[i], angle);
        snprintf(row, sizeof(row), "%-5s %6.6s    ", labels[i], angle);
        OLED_ShowString(i + 2, 1, row);
    }
}

void DisplayApp_Poll(void) { OLED_Poll(); }
