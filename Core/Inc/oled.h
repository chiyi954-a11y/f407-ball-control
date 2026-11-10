#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ============================================================
 * SSD1306 128x64，软件 I2C
 * ------------------------------------------------------------
 * SCL = PB6
 * SDA = PB7
 * 模块一般自带 4.7k 上拉；若没有，需要外接。
 * ============================================================ */

#define OLED_I2C_ADDR   0x78    /* 0x3C << 1 */
#define OLED_WIDTH      128
#define OLED_PAGES      8

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Fill(uint8_t data);
void OLED_Refresh(void);
void OLED_ShowChar(uint8_t page, uint8_t x, char ch);
void OLED_ShowString(uint8_t page, uint8_t x, const char *str);
void OLED_ShowNum(uint8_t page, uint8_t x, int32_t num, uint8_t width);

/* 2 倍字号：每字 12x16 像素，占两个 page（page 只能是 0/2/4/6） */
void OLED_ShowChar2x(uint8_t page, uint8_t x, char ch);
void OLED_ShowString2x(uint8_t page, uint8_t x, const char *str);
void OLED_ShowNum2x(uint8_t page, uint8_t x, int32_t num, uint8_t width);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
