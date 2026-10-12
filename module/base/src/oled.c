#include "oled.h"
#include "OLED_Font.h"
#include <string.h>

/* SSD1306 128×64 屏幕，采用页寻址；每次轮询最多发送一页中的 16 列数据。 */
static I2C_HandleTypeDef *oled_bus;
static uint8_t frame[1024], dirty[64];
static uint8_t ready, address, next_chunk;
static uint32_t error, transfers;

static HAL_StatusTypeDef send_bytes(uint8_t *data, uint16_t count)
{
    HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(oled_bus,
        (uint16_t)address << 1, data, count, 10);
    if (result != HAL_OK) {
        error = HAL_I2C_GetError(oled_bus);
        ready = 0;
    }
    return result;
}

HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *bus)
{
    ready = 0;
    address = 0;
    oled_bus = bus;
    if (!bus) return HAL_ERROR;
    for (uint8_t candidate = 0x3C; candidate <= 0x3D; ++candidate) {
        if (HAL_I2C_IsDeviceReady(bus, (uint16_t)candidate << 1, 1, 10) == HAL_OK) {
            address = candidate;
            break;
        }
    }
    if (!address) {
        error = HAL_I2C_GetError(bus);
        return HAL_ERROR;
    }
    /* 沿用原驱动的屏幕参数，并显式设置为页寻址模式。 */
    uint8_t commands[] = {0x00, 0xAE, 0xD5, 0x80, 0xA8, 0x3F,
        0xD3, 0x00, 0x40, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF,
        0xD9, 0xF1, 0xDB, 0x30, 0xA4, 0xA6, 0x8D, 0x14, 0x20, 0x02, 0xAF};
    if (send_bytes(commands, sizeof(commands)) != HAL_OK) return HAL_ERROR;
    error = 0;
    ready = 1;
    next_chunk = 0;
    OLED_Clear();
    return HAL_OK;
}

uint8_t OLED_IsReady(void) { return ready; }
uint8_t OLED_Address(void) { return address; }
uint32_t OLED_Error(void) { return error; }
uint32_t OLED_Transfers(void) { return transfers; }

void OLED_Clear(void)
{
    memset(frame, 0, sizeof(frame));
    memset(dirty, 1, sizeof(dirty));
}

void OLED_Poll(void)
{
    if (!ready) return;
    for (unsigned n = 0; n < 64; ++n) {
        uint8_t chunk = next_chunk;
        next_chunk = (next_chunk + 1) % 64;
        if (!dirty[chunk]) continue;
        uint8_t page = chunk / 8, column = (chunk % 8) * 16;
        uint8_t cursor[] = {0x00, (uint8_t)(0xB0 | page),
            (uint8_t)(0x10 | (column >> 4)), (uint8_t)(column & 0x0F)};
        uint8_t data[17] = {0x40};
        memcpy(data + 1, frame + chunk * 16, 16);
        if (send_bytes(cursor, sizeof(cursor)) != HAL_OK) return;
        if (send_bytes(data, sizeof(data)) != HAL_OK) return;
        dirty[chunk] = 0;
        transfers++;
        return;
    }
}

void OLED_ShowChar(uint8_t line, uint8_t column, char character)
{
    if (line < 1 || line > 4 || column < 1 || column > 16) return;
    unsigned index = (unsigned char)character;
    if (index < 32 || index - 32 >= sizeof(OLED_F8x16) / sizeof(OLED_F8x16[0]))
        index = '?';
    index -= 32;
    for (unsigned half = 0; half < 2; ++half) {
        unsigned offset = ((line - 1) * 2 + half) * 128 + (column - 1) * 8;
        if (memcmp(frame + offset, OLED_F8x16[index] + half * 8, 8)) {
            memcpy(frame + offset, OLED_F8x16[index] + half * 8, 8);
            dirty[offset / 16] = 1;
        }
    }
}

void OLED_ShowString(uint8_t line, uint8_t column, const char *str)
{
    if (!str || line < 1 || line > 4 || column < 1 || column > 16) return;
    while (*str && column <= 16) OLED_ShowChar(line, column++, *str++);
}

static void show_unsigned(uint8_t line, uint8_t column, uint32_t value,
                          uint8_t length, unsigned base)
{
    static const char digits[] = "0123456789ABCDEF";
    char text[17];
    if (line < 1 || line > 4 || column < 1 || column > 16) return;
    if (length > 17 - column) length = 17 - column;
    text[length] = '\0';
    for (unsigned i = length; i > 0; --i) {
        text[i - 1] = digits[value % base];
        value /= base;
    }
    OLED_ShowString(line, column, text);
}
void OLED_ShowNum(uint8_t l, uint8_t c, uint32_t n, uint8_t len)
{ show_unsigned(l, c, n, len, 10); }
void OLED_ShowHexNum(uint8_t l, uint8_t c, uint32_t n, uint8_t len)
{ show_unsigned(l, c, n, len, 16); }
void OLED_ShowBinNum(uint8_t l, uint8_t c, uint32_t n, uint8_t len)
{ show_unsigned(l, c, n, len, 2); }
void OLED_ShowSignedNum(uint8_t l, uint8_t c, int32_t n, uint8_t len)
{
    if (l < 1 || l > 4 || c < 1 || c > 16) return;
    OLED_ShowChar(l, c, n < 0 ? '-' : '+');
    uint32_t magnitude = n < 0 ? 0U - (uint32_t)n : (uint32_t)n;
    show_unsigned(l, c + 1, magnitude, len, 10);
}
