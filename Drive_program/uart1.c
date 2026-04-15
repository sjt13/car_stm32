
#include "main.h"

/**
 * @简  述  UART 串口初始化
 * @参  数  baud： 波特率设置
 * @返回值	 无
 */
// 串口1，自动下载电路
void UART1_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 串口 USART1配置 */
	// 打开GPIO和USART部件的时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// USART1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	// USART1 端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// USART1参数配置
	USART_InitStructure.USART_BaudRate = baud; // 波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	// 使能 USART， 配置完毕
	USART_Cmd(USART1, ENABLE);

	// 规避第一个字符不能输出的BUG
	//	USART_ClearFlag(USART1, USART_FLAG_TC);

	// 开启串口接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启相关中断

	// Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // 串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // 子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器
}


/***********************************************
	功能介绍：	串口1发送字节
	函数参数：	dat 发送的字节
	返回值：		无
 ***********************************************/
void uart1_send_byte(u8 dat)
{
	USART_SendData(USART1, dat);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
}

/***********************************************
	功能介绍：	串口1发送字符串
	函数参数：	*s 发送的字符串
	返回值：		无
 ***********************************************/
void uart1_send_str(u8 *s)
{
	while (*s)
	{
		uart1_send_byte(*s++);
	}
}

/* QSA */
/**
 * @简  述  DBUART 串口中断服务函数
 * @参  数  无
 * @返回值  无
 */
void USART1_IRQHandler(void)
{
   u8 sbuf_bak;
   static u16 buf_index = 0;
   if (USART_GetFlagStatus(USART1, USART_IT_RXNE) == SET)
   {
       USART_ClearITPendingBit(USART1, USART_IT_RXNE);
       sbuf_bak = USART_ReceiveData(USART1);
       //USART_SendData(USART1, sbuf_bak);
       if (uart1_get_ok)
           return;
       if (sbuf_bak == '<')
       {
           uart1_mode = 4;
           buf_index = 0;
       }
       else if (uart1_mode == 0)
       {
           if (sbuf_bak == '$')
           {
               uart1_mode = 1;
           }
           else if (sbuf_bak == '#')
           {
               uart1_mode = 2;
           }
           else if (sbuf_bak == '{')
           {
               uart1_mode = 3;
           }
           else if (sbuf_bak == '<')
           {
               uart1_mode = 4;
           }
           buf_index = 0;
       }

       uart_receive_buf[buf_index++] = sbuf_bak;

       if ((uart1_mode == 4) && (sbuf_bak == '>'))
       {
           uart_receive_buf[buf_index] = '\0';
           uart1_get_ok = 1;
       }
       else if ((uart1_mode == 1) && (sbuf_bak == '!'))
       {
           uart_receive_buf[buf_index] = '\0';
           uart1_get_ok = 1;
       }
       else if ((uart1_mode == 2) && (sbuf_bak == '!'))
       {
           uart_receive_buf[buf_index] = '\0';
           uart1_get_ok = 1;
       }
       else if ((uart1_mode == 3) && (sbuf_bak == '}'))
       {
           uart_receive_buf[buf_index] = '\0';
           uart1_get_ok = 1;
       }
		
	    if(uart1_get_ok==1)
	    {
			uart_receive_num = 1;
        }
		
       if (buf_index >= 1024)
           buf_index = 0;
   }
}


/**************************串口打印相关函数重定义********************************/
/**
 * @简  述  重定义putc函数（USART1）
 */
int fputc(int ch, FILE *f)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (uint8_t)ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}

	return ch;
}

/**
 * @简  述  重定义getc函数（USART1）
 */
int fgetc(FILE *f)
{
	/* 等待串口1输入数据 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
	{
	}

	return (int)USART_ReceiveData(USART1);
}

/******************* (C) 版权 2022 XTARK **************************************/
