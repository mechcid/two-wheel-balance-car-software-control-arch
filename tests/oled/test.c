#include "oled.h"
#include <assert.h>
#include <stdio.h>
static unsigned blocks, cursor, hits[128];
static int fail;
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *b,uint16_t a,uint32_t n,uint32_t t)
{ (void)b;(void)n;(void)t;return a==0x78 ? HAL_OK:HAL_ERROR; }
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *b) { (void)b;return 4; }
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *b,uint16_t a,uint8_t *d,uint16_t n,uint32_t t)
{
    (void)b;(void)t;assert(a==0x78);
    if (fail) return HAL_ERROR;
    if (n==4) { unsigned page=d[1]&7, col=((d[2]&15)<<4)|(d[3]&15); assert(col%8==0); cursor=page*16+col/8; }
    if (d[0]==0x40) { assert(n==9 && cursor<128);++hits[cursor];++blocks; }
    return HAL_OK;
}
int main(void) {
    I2C_HandleTypeDef b={0};assert(OLED_Init(&b)==HAL_OK);
    for (unsigned i=0;i<128;++i) OLED_Poll();
    assert(blocks==128);for(unsigned i=0;i<128;++i) assert(hits[i]==1);
    OLED_Poll();assert(blocks==128);
    OLED_ShowChar(1,1,'A');OLED_ShowChar(4,16,'B');
    for(unsigned i=0;i<4;++i) OLED_Poll();
    assert(blocks==132 && hits[0]==2 && hits[16]==2 && hits[111]==2 && hits[127]==2);
    OLED_ShowChar(1,1,'A');OLED_Poll();assert(blocks==132);
    OLED_Clear();fail=1;OLED_Poll();assert(!OLED_IsReady() && OLED_Error()==4);
    fail=0;assert(OLED_Init(&b)==HAL_OK);for(unsigned i=0;i<128;++i) OLED_Poll();
    assert(blocks==260);puts("PASS: OLED 8-byte full coverage, character dirty marking, unchanged pixels, failure/reinit");
}
