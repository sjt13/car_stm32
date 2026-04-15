#include "y_zx_uart3.h"

/************************************************
函数名称 ： USART3_Send_ArrayU8
功    能 ： 总线舵机的串口设置
参    数 ： pData ---- 字符串
            Length --- 长度
返 回 值 ： 无
*************************************************/
/* QSA */
void USART3_Init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 串口 USART1配置 */
    // 打开GPIO和USART部件的时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // USART3对应引脚复用映射
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

    // USART3 端口配置
    // USART 端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //开漏上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART3参数配置
    USART_InitStructure.USART_BaudRate = baud; // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
	USART_HalfDuplexCmd(USART3, ENABLE);  	//注意这个，启动半双工模式

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;         // 串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                           // 根据指定的参数初始化VIC寄存器

	// 开启串口接收中断
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启相关中断
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE); /* 禁止串口发送中断 */
	// 使能 USART， 配置完毕
    USART_Cmd(USART3, ENABLE);
}

/***********************************************
	函数名称:	uart3_send_str()
	功能介绍:	串口3发送字符串
	函数参数:	*s 发送的字符串
	返回值:		无
 ***********************************************/
void uart3_send_str(uint8_t *s)
{
	//USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
	//sendOnly(USART3);
	while (*s)
	{
		USART_SendData(USART3, *s++);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
			;
	}
//	USART_SendData(USART3, 0xff);
//	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
//		;
	//readOnly(USART3);
	//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/*  串口中断接收处理 */
void USART3_IRQHandler(void)
{
    u8 sbuf_bak;
	static u16 buf_index = 0;
	if (USART_GetFlagStatus(USART3, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		sbuf_bak = USART_ReceiveData(USART3);
		//USART_SendData(USART1, sbuf_bak);
		if (uart1_get_ok)
			return;

		if (uart1_mode == 0)
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
			buf_index = 0;
		}

		uart_receive_buf[buf_index++] = sbuf_bak;


		if ((uart1_mode == 1) && (sbuf_bak == '!'))
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
			uart_receive_num = 3;
		}

		if (buf_index >= 1024)
			buf_index = 0;
	}
}
