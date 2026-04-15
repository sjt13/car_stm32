
#ifndef __UART6_H
#define __UART6_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#include "y_global.h"

void UART6_Init(uint32_t baud); // UART 调试串口初始化
void uart6_send_str(u8 *s);
void uart6_send_byte(u8 dat);

#endif

/******************* (C) 版权 2022 XTARK **************************************/
