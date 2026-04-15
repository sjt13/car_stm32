
#ifndef __UART4_H
#define __UART4_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#include "y_global.h"

void UART4_Init(uint32_t baud); // UART 调试串口初始化
void uart4_send_str(u8 *s);

#endif

/******************* (C) 版权 2022 XTARK **************************************/
