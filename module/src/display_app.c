#include "display_app.h"
#include "oled.h"
#include <stdio.h>

Display display= gravity;

HAL_StatusTypeDef DisplayApp_Init(I2C_HandleTypeDef *bus){
    HAL_StatusTypeDef result = OLED_Init(bus);
    if (result == HAL_OK) OLED_ShowString(1, 1, "MPU6050 / mg    ");
    return result;
}
void DisplayApp_Render(const SensorSample *sample, bool available){
    if (!OLED_IsReady()) return;
    if (!sample || !sample->valid || !available) {
        OLED_ShowString(2, 1, "MPU waiting...  ");
        OLED_ShowString(3, 1, "                ");
        OLED_ShowString(4, 1, "                ");
        return;
    }
    const float axes[] = {sample->ax_g, sample->ay_g, sample->az_g};
    const float degree[] = {sample->gx_dps, sample->gy_dps, sample->gz_dps};
    for (unsigned i = 0; i < 3; ++i) {
        char row[17];

        switch (display){
        case gravity:
            snprintf(row, sizeof(row), "A%c %+7ld mg  ", "XYZ"[i], (long)(axes[i] * 1000.0f));
            break;
        case angular:
            snprintf(row, sizeof(row), "G%c %+7ld mdps ", "XYZ"[i], (long)(degree[i]* 1000.0f));
            break;
        default:
            break;
        }
        
        OLED_ShowString(i + 2, 1, row);
    }
}
void DisplayApp_Poll(void) { OLED_Poll(); }
