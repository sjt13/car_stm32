/******************************************************************************
 * 文件名  : y_oled.c
 * 说明    : STM32F407VET6 + OLED(SSD1306 类) 软件I2C驱动
 * 功能    :
 *          1. OLED 初始化
 *          2. 写命令 / 写数据
 *          3. 显示单个字符
 *          4. 显示字符串
 *          5. 显示无符号数字
 *          6. 显示有符号数字
 *          7. 显示十六进制 / 二进制
 *          8. 显示 16x16 汉字 / 图片
 *
 * 说明补充:
 *          - 本驱动基于你原来的软件I2C代码整理优化
 *          - 保留旧接口，方便兼容已有工程
 *          - 字库数据依赖 resource.h 中的 F6x8、F8X16、F16x16、FONT_COLOR、BMP1 等
 ******************************************************************************/

#include "y_oled.h"
#include "resource.h"
#include <stdio.h>      /* snprintf */
#include <string.h>     /* strlen */

#define I2C_TIMEOUT_TIMES   100

/*========================================================
 * GPIO 相关配置
 * 原工程使用:
 *   SCL -> PA8
 *   SDA -> PC9
 *=======================================================*/

/**
 * @brief  软件I2C使用的GPIO初始化
 * @note   这里采用开漏输出 + 上拉，符合I2C总线要求
 */
static void soft_i2c_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

    /* SCL: PA8 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      /* 开漏输出 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       /* 上拉 */
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* SDA: PC9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 * @brief  I2C超时用的最小延时
 * @note   依赖你工程里的 Delay_us()
 */
static void i2c_timeout_delay(void)
{
    Delay_us(1);
}

/**
 * @brief  I2C每个阶段之间的短延时
 * @note   用于等待总线电平稳定，同时控制通信速度
 */
static void i2c_delay(void)
{
    uint8_t i = 2;
    while (--i)
    {
        i2c_timeout_delay();
    }
}

/*-------------------- I2C引脚电平控制 --------------------*/
static void I2C_SCL_H(void) { GPIO_SetBits(GPIOA, GPIO_Pin_8); }
static void I2C_SCL_L(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_8); }
static void I2C_SDA_H(void) { GPIO_SetBits(GPIOC, GPIO_Pin_9); }
static void I2C_SDA_L(void) { GPIO_ResetBits(GPIOC, GPIO_Pin_9); }

/**
 * @brief  读取 SDA 引脚电平
 * @retval 0: 低电平  1: 高电平
 */
static uint8_t I2C_SDA_Read(void)
{
    return (uint8_t)GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9);
}

/*========================================================
 * 软件 I2C 底层
 *=======================================================*/

/**
 * @brief  产生 I2C 起始信号
 * @note   当 SCL 为高时，SDA 从高跳变到低
 */
static void i2c_start(void)
{
    I2C_SDA_H();
    I2C_SCL_H();
    i2c_delay();

    I2C_SDA_L();
    i2c_delay();
    I2C_SCL_L();
}

/**
 * @brief  产生 I2C 停止信号
 * @note   当 SCL 为高时，SDA 从低跳变到高
 */
static void i2c_stop(void)
{
    I2C_SDA_L();
    I2C_SCL_H();
    i2c_delay();

    I2C_SDA_H();
    i2c_delay();
}

/**
 * @brief  等待从机应答
 * @retval 0: 应答成功
 * @retval 1: 应答失败/超时
 */
static uint8_t i2c_wait_ack(void)
{
    uint16_t timeout = 0;

    I2C_SCL_H();
    I2C_SDA_H();
    i2c_delay();

    while (I2C_SDA_Read())
    {
        timeout++;
        i2c_timeout_delay();

        if (timeout > I2C_TIMEOUT_TIMES)
        {
            i2c_stop();
            return 1;
        }
    }

    I2C_SCL_L();
    return 0;
}

/**
 * @brief  I2C 发送一个字节
 * @param  dat 要发送的数据
 * @note   高位先发
 */
static void i2c_write_byte(uint8_t dat)
{
    uint8_t i;

    I2C_SCL_L();
    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80)
            I2C_SDA_H();
        else
            I2C_SDA_L();

        dat <<= 1;
        i2c_delay();

        I2C_SCL_H();
        i2c_delay();

        I2C_SCL_L();
        i2c_delay();
    }
}

/**
 * @brief  向 OLED 写入一个控制字节 + 一个数据字节
 * @param  control  控制字节，0x00 表示后面是命令，0x40 表示后面是数据
 * @param  data     实际要写入 OLED 的字节
 */
static void OLED_I2C_WriteByte(uint8_t control, uint8_t data)
{
    i2c_start();

    i2c_write_byte(OLED_ADDRESS);
    if (i2c_wait_ack())
    {
        i2c_stop();
        return;
    }

    i2c_write_byte(control);
    if (i2c_wait_ack())
    {
        i2c_stop();
        return;
    }

    i2c_write_byte(data);
    if (i2c_wait_ack())
    {
        i2c_stop();
        return;
    }

    i2c_stop();
}

/*========================================================
 * 基础写命令 / 写数据接口
 *=======================================================*/

/**
 * @brief  向 OLED 写入一个数据字节
 * @param  data 数据
 */
void OLED_WriteDat(uint8_t data)
{
    OLED_I2C_WriteByte(0x40, data);
}

/**
 * @brief  向 OLED 写入一个命令字节
 * @param  cmd 命令
 */
void OLED_WriteCmd(uint8_t cmd)
{
    OLED_I2C_WriteByte(0x00, cmd);
}

/* 为兼容旧函数名保留 */
void OLED_Write_Dat(uint8_t data) { OLED_WriteDat(data); }
void OLED_Write_Cmd(uint8_t cmd)  { OLED_WriteCmd(cmd); }

/*========================================================
 * OLED 基础控制
 *=======================================================*/

/**
 * @brief  设置 OLED 显示光标位置
 * @param  x 列地址，范围 0~127
 * @param  y 页地址，范围 0~7
 */
void OLED_SetPos(uint8_t x, uint8_t y)
{
    if (x >= OLED_WIDTH) return;
    if (y >= OLED_PAGE_NUM) return;

    OLED_WriteCmd(0xB0 + y);
    OLED_WriteCmd(((x & 0xF0) >> 4) | 0x10);
    OLED_WriteCmd((x & 0x0F) | 0x00);
}

/* 为兼容旧函数名保留 */
void OLED_Set_Pos(uint8_t x, uint8_t y) { OLED_SetPos(x, y); }

/**
 * @brief  OLED 全屏填充
 * @param  fill_data 每个字节写入的数据
 *         0x00 -> 全灭
 *         0xFF -> 全亮
 */
void OLED_Fill(uint8_t fill_data)
{
    uint8_t x, y;

    for (y = 0; y < OLED_PAGE_NUM; y++)
    {
        OLED_SetPos(0, y);
        for (x = 0; x < OLED_WIDTH; x++)
        {
            OLED_WriteDat(fill_data);
        }
    }
}

/**
 * @brief  清屏
 */
void OLED_Clear(void)
{
    OLED_Fill(0x00);
}

/* 兼容旧接口 */
void OLED_CLS(void) { OLED_Clear(); }

/**
 * @brief  清除某一页（某一行）
 * @param  line 页号，0~7
 * @note   6x8 字体中，1页相当于1行
 *         8x16 字体中，1个字符会占2页
 */
void OLED_ClearLine(uint8_t line)
{
    uint8_t x;

    if (line >= OLED_PAGE_NUM) return;

    OLED_SetPos(0, line);
    for (x = 0; x < OLED_WIDTH; x++)
    {
        OLED_WriteDat(0x00);
    }
}

/**
 * @brief  打开 OLED 显示
 */
void OLED_ON(void)
{
    OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14);
    OLED_WriteCmd(0xAF);
}

/**
 * @brief  关闭 OLED 显示
 */
void OLED_OFF(void)
{
    OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x10);
    OLED_WriteCmd(0xAE);
}

/**
 * @brief  OLED 初始化
 * @note   适用于常见 SSD1306/兼容控制器的 128x64 OLED
 */
void OLED_Init(void)
{
    soft_i2c_gpio_init();

    OLED_WriteCmd(0xAE);                  /* Display OFF */

    OLED_WriteCmd(0x00);                  /* Low column address */
    OLED_WriteCmd(0x10);                  /* High column address */
    OLED_WriteCmd(0x40);                  /* Start line address */

    OLED_WriteCmd(0x81);                  /* Contrast control */
    OLED_WriteCmd(OLED_BRIGHTNESS);

    OLED_WriteCmd(0xA1);                  /* Segment remap */
    OLED_WriteCmd(0xC8);                  /* COM scan direction */

    OLED_WriteCmd(0xA6);                  /* Normal display */

    OLED_WriteCmd(0xA8);                  /* Multiplex ratio */
    OLED_WriteCmd(0x3F);                  /* 1/64 duty */

    OLED_WriteCmd(0xD3);                  /* Display offset */
    OLED_WriteCmd(0x00);

    OLED_WriteCmd(0xD5);                  /* Display clock divide */
    OLED_WriteCmd(0x80);

    OLED_WriteCmd(0xD9);                  /* Pre-charge period */
    OLED_WriteCmd(0xF1);

    OLED_WriteCmd(0xDA);                  /* COM pins hardware config */
    OLED_WriteCmd(0x12);

    OLED_WriteCmd(0xDB);                  /* VCOMH deselect level */
    OLED_WriteCmd(0x40);

    OLED_WriteCmd(0x20);                  /* Memory addressing mode */
    OLED_WriteCmd(0x02);                  /* Page addressing mode */

    OLED_WriteCmd(0x8D);                  /* Charge pump */
    OLED_WriteCmd(0x14);

    OLED_WriteCmd(0xA4);                  /* Output follows RAM content */
    OLED_WriteCmd(0xA6);                  /* Non-inverted display */

    OLED_WriteCmd(0xAF);                  /* Display ON */

    OLED_Clear();
    OLED_SetPos(0, 0);
}

/*========================================================
 * 内部工具函数
 *=======================================================*/

/**
 * @brief  计算整数次幂
 * @param  x 底数
 * @param  y 指数
 * @retval x^y
 */
static uint32_t OLED_Pow(uint32_t x, uint32_t y)
{
    uint32_t result = 1;
    while (y--)
    {
        result *= x;
    }
    return result;
}

/**
 * @brief  根据字体大小获取字符宽度
 */
static uint8_t OLED_GetFontWidth(uint8_t font_size)
{
    return (font_size == OLED_FONT_8X16) ? 8 : 6;
}

/*========================================================
 * 字符 / 字符串显示
 *=======================================================*/

/**
 * @brief  显示一个 ASCII 字符
 * @param  x         起始列坐标
 * @param  y         起始页坐标
 * @param  chr       要显示的字符
 * @param  font_size 字体大小
 *                   OLED_FONT_6X8  -> 6x8
 *                   OLED_FONT_8X16 -> 8x16
 *
 * @note   支持可打印 ASCII 字符（' ' ~ '~'）
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t font_size)
{
    uint8_t c;
    uint8_t i;

    if (chr < ' ' || chr > '~')
    {
        chr = '?';
    }
    c = (uint8_t)(chr - ' ');

    if (font_size == OLED_FONT_6X8)
    {
        if (x > OLED_WIDTH - 6 || y > 7) return;

        OLED_SetPos(x, y);
        for (i = 0; i < 6; i++)
        {
            OLED_WriteDat(F6x8[c][i]);
        }
    }
    else if (font_size == OLED_FONT_8X16)
    {
        if (x > OLED_WIDTH - 8 || y > 6) return;

        OLED_SetPos(x, y);
        for (i = 0; i < 8; i++)
        {
            OLED_WriteDat(F8X16[c * 16 + i]);
        }

        OLED_SetPos(x, y + 1);
        for (i = 0; i < 8; i++)
        {
            OLED_WriteDat(F8X16[c * 16 + i + 8]);
        }
    }
}

/**
 * @brief  显示字符串
 * @param  x         起始列坐标
 * @param  y         起始页坐标
 * @param  str       字符串首地址
 * @param  font_size 字体大小
 *
 * @note   这里实现了简单自动换行：
 *         - 6x8 字体宽 6
 *         - 8x16 字体宽 8
 *         超出一行后从下一页/下两页继续显示
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t font_size)
{
    uint8_t char_width;

    if (str == NULL) return;

    char_width = OLED_GetFontWidth(font_size);

    while (*str != '\0')
    {
        /* 遇到换行符，手动换到下一行 */
        if (*str == '\n')
        {
            x = 0;
            y += (font_size == OLED_FONT_8X16) ? 2 : 1;
            str++;
            continue;
        }

        /* 自动换行 */
        if (x > (OLED_WIDTH - char_width))
        {
            x = 0;
            y += (font_size == OLED_FONT_8X16) ? 2 : 1;
        }

        /* 越界保护 */
        if ((font_size == OLED_FONT_6X8  && y > 7) ||
            (font_size == OLED_FONT_8X16 && y > 6))
        {
            break;
        }

        OLED_ShowChar(x, y, *str, font_size);
        x += char_width;
        str++;
    }
}

/*========================================================
 * 数字显示
 *=======================================================*/

/**
 * @brief  显示无符号十进制数字
 * @param  x         起始列坐标
 * @param  y         起始页坐标
 * @param  num       要显示的数字
 * @param  len       显示长度（位数）
 * @param  font_size 字体大小
 *
 * @example
 *   OLED_ShowNum(0, 0, 123, 5, OLED_FONT_8X16);
 *   实际显示: "00123"
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t font_size)
{
    uint8_t i;
    uint8_t digit;
    uint32_t div;

    for (i = 0; i < len; i++)
    {
        div = OLED_Pow(10, len - i - 1);
        digit = (num / div) % 10;
        OLED_ShowChar(x + i * OLED_GetFontWidth(font_size), y, digit + '0', font_size);
    }
}

/**
 * @brief  显示有符号十进制数字
 * @param  x         起始列坐标
 * @param  y         起始页坐标
 * @param  num       要显示的数字
 * @param  len       数值部分长度（不含符号位）
 * @param  font_size 字体大小
 *
 * @example
 *   OLED_ShowSignedNum(0,0,-25,3,OLED_FONT_8X16);
 *   显示: -025
 */
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t font_size)
{
    uint32_t abs_num;
    uint8_t width = OLED_GetFontWidth(font_size);

    if (num >= 0)
    {
        OLED_ShowChar(x, y, '+', font_size);
        abs_num = (uint32_t)num;
    }
    else
    {
        OLED_ShowChar(x, y, '-', font_size);
        abs_num = (uint32_t)(-num);
    }

    OLED_ShowNum(x + width, y, abs_num, len, font_size);
}

/**
 * @brief  显示十六进制数字
 * @param  x         起始列坐标
 * @param  y         起始页坐标
 * @param  num       要显示的数字
 * @param  len       显示长度（十六进制位数）
 * @param  font_size 字体大小
 *
 * @example
 *   OLED_ShowHexNum(0,0,0x1AF,4,OLED_FONT_6X8);
 *   显示: 01AF
 */
void OLED_ShowHexNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t font_size)
{
    uint8_t i;
    uint8_t single_num;
    uint8_t width = OLED_GetFontWidth(font_size);

    for (i = 0; i < len; i++)
    {
        single_num = (num / OLED_Pow(16, len - i - 1)) % 16;

        if (single_num < 10)
            OLED_ShowChar(x + i * width, y, single_num + '0', font_size);
        else
            OLED_ShowChar(x + i * width, y, single_num - 10 + 'A', font_size);
    }
}

/**
 * @brief  显示二进制数字
 * @param  x         起始列坐标
 * @param  y         起始页坐标
 * @param  num       要显示的数字
 * @param  len       显示长度（二进制位数）
 * @param  font_size 字体大小
 */
void OLED_ShowBinNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t font_size)
{
    uint8_t i;
    uint8_t width = OLED_GetFontWidth(font_size);

    for (i = 0; i < len; i++)
    {
        if ((num / OLED_Pow(2, len - i - 1)) % 2)
            OLED_ShowChar(x + i * width, y, '1', font_size);
        else
            OLED_ShowChar(x + i * width, y, '0', font_size);
    }
}

/*========================================================
 * 汉字 / 彩色字模 / 图片
 *=======================================================*/

/**
 * @brief  显示 16x16 汉字
 * @param  x     起始列坐标
 * @param  y     起始页坐标
 * @param  index 汉字在字模表 F16x16 中的序号
 *
 * @note   每个 16x16 汉字占 32 字节，上半部分16字节，下半部分16字节
 */
void OLED_ShowChinese16x16(uint8_t x, uint8_t y, uint8_t index)
{
    uint8_t i;
    uint16_t addr = 32 * index;

    if (x > OLED_WIDTH - 16 || y > 6) return;

    OLED_SetPos(x, y);
    for (i = 0; i < 16; i++)
    {
        OLED_WriteDat(F16x16[addr++]);
    }

    OLED_SetPos(x, y + 1);
    for (i = 0; i < 16; i++)
    {
        OLED_WriteDat(F16x16[addr++]);
    }
}

/* 保留兼容旧接口 */
void OLED_P16x16Ch(uint8_t x, uint8_t y, uint8_t N)
{
    OLED_ShowChinese16x16(x, y, N);
}

/**
 * @brief  显示彩色字模（依赖 FONT_COLOR 字库）
 * @param  x     起始列坐标
 * @param  y     起始页坐标
 * @param  index 字模索引
 */
void OLED_ShowColorFont(uint8_t x, uint8_t y, uint8_t index)
{
    uint8_t i;
    uint16_t addr = 2 * index;

    if (x > OLED_WIDTH - 16 || y > 6) return;

    OLED_SetPos(x, y);
    for (i = 0; i < 16; i++)
    {
        OLED_WriteDat(FONT_COLOR[addr][i]);
    }

    OLED_SetPos(x, y + 1);
    for (i = 0; i < 16; i++)
    {
        OLED_WriteDat(FONT_COLOR[addr + 1][i]);
    }
}

/**
 * @brief  显示 BMP 位图数据
 * @param  x0   起始列
 * @param  y0   起始页
 * @param  x1   结束列（不包含）
 * @param  y1   结束页（包含）
 * @param  bmp  位图数据首地址
 * @param  len  位图数据长度
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0,
                  uint8_t x1, uint8_t y1,
                  const uint8_t *bmp, uint16_t len)
{
    uint16_t j = 0;
    uint8_t x, y;

    if (bmp == NULL) return;
    if (x0 >= OLED_WIDTH || x1 > OLED_WIDTH || y0 >= OLED_PAGE_NUM || y1 >= OLED_PAGE_NUM) return;
    if (x0 >= x1 || y0 > y1) return;

    for (y = y0; y <= y1; y++)
    {
        OLED_SetPos(x0, y);
        for (x = x0; x < x1; x++)
        {
            if (j >= len)
                return;
            OLED_WriteDat(bmp[j++]);
        }
    }
}

/*========================================================
 * 兼容旧版字符串接口
 *=======================================================*/
void OLED_P6x8Str(uint8_t x, uint8_t y, uint8_t ch[])
{
    OLED_ShowString(x, y, (const char *)ch, OLED_FONT_6X8);
}

void OLED_P8x16Str(uint8_t x, uint8_t y, uint8_t ch[])
{
    OLED_ShowString(x, y, (const char *)ch, OLED_FONT_8X16);
}

void OLED_ShowStr(uint8_t x, uint8_t y, uint8_t ch[], uint8_t TextSize)
{
    OLED_ShowString(x, y, (const char *)ch, TextSize);
}

/*========================================================
 * 测试函数
 *=======================================================*/
void OLED_TEST(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "OLED TEST", OLED_FONT_8X16);
    OLED_ShowString(0, 2, "STM32F407VET6", OLED_FONT_6X8);
    OLED_ShowNum(0, 4, 12345, 5, OLED_FONT_6X8);
    OLED_DrawBMP(0, 6, 128, 7, BMP1, sizeof(BMP1));
}