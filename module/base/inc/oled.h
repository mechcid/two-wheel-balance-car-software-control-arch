#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SSD1306 128x64, 0x3C/0x3D. Coordinates: lines 1..4, columns 1..16.
 * Show functions update RAM; call OLED_Poll regularly to send dirty chunks. */
HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *bus);
void OLED_Poll(void);
uint8_t OLED_IsReady(void);
uint8_t OLED_Address(void);
uint32_t OLED_Error(void);
uint32_t OLED_Transfers(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#ifdef __cplusplus
}
#endif

#endif
