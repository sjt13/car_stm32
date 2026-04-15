#ifndef __Y_OLED_H
#define __Y_OLED_H

#include "main.h"
#include <stdint.h>

/*========================================================
 * OLED 硬件配置
 *=======================================================*/
/*
 * OLED 的 I2C 7位地址通常为 0x3C / 0x3D
 * 在很多程序里会写成“8位地址”，即左移1位后的 0x78 / 0x7A
 * 本驱动沿用原工程写法：使用 8 位地址 0x78
 */
#define OLED_ADDRESS        0x78

/* OLED 屏幕参数 */
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_PAGE_NUM       8       /* 64 / 8 = 8 页 */
#define OLED_BRIGHTNESS     0xCF

/* 字体大小选择 */
#define OLED_FONT_6X8       1
#define OLED_FONT_8X16      2

#ifdef __cplusplus
extern "C" {
#endif

/*========================================================
 * 基础底层接口
 *=======================================================*/
void OLED_WriteDat(uint8_t data);
void OLED_WriteCmd(uint8_t cmd);

void OLED_ON(void);
void OLED_OFF(void);

void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_Fill(uint8_t fill_data);
void OLED_Clear(void);
void OLED_ClearLine(uint8_t line);

void OLED_Init(void);

/*========================================================
 * 基础显示接口
 *=======================================================*/
/*
 * x:
 *   - 6x8 字体时，建议范围 0~127
 *   - 8x16 字体时，建议范围 0~127
 * y:
 *   - 6x8 字体时，y 为页号，范围 0~7
 *   - 8x16 字体时，字符占两页，建议 y 范围 0~6
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t font_size);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t font_size);
void OLED_ShowChinese16x16(uint8_t x, uint8_t y, uint8_t index);
void OLED_ShowColorFont(uint8_t x, uint8_t y, uint8_t index);

void OLED_DrawBMP(uint8_t x0, uint8_t y0,
                  uint8_t x1, uint8_t y1,
                  const uint8_t *bmp, uint16_t len);

/*========================================================
 * 数字显示接口
 *=======================================================*/
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t font_size);
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t font_size);
void OLED_ShowHexNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t font_size);
void OLED_ShowBinNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t font_size);

/*========================================================
 * 兼容旧接口（保留，方便你原工程继续使用）
 *=======================================================*/
void OLED_P6x8Str(uint8_t x, uint8_t y, uint8_t ch[]);
void OLED_P8x16Str(uint8_t x, uint8_t y, uint8_t ch[]);
void OLED_P16x16Ch(uint8_t x, uint8_t y, uint8_t N);
void OLED_ShowStr(uint8_t x, uint8_t y, uint8_t ch[], uint8_t TextSize);
void OLED_CLS(void);
void OLED_Write_Dat(uint8_t data);
void OLED_Write_Cmd(uint8_t cmd);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_TEST(void);

#ifdef __cplusplus
}
#endif

#endif