

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART1_H
#define __UART1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#include "y_global.h"

void UART1_Init(uint32_t baud); 
void uart1_send_str(u8 *s);
void uart1_send_byte(u8 dat);

#endif 

/******************* (C 2019 XTARK **************************************/
