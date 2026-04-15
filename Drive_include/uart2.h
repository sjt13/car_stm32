
#ifndef __UART2_H
#define __UART2_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

void UART2_Init(uint32_t baud);
void uart2_send_str(u8 *s);
#endif
