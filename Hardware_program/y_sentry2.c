#include "y_sentry2.h"
#include "uart6.h"

uint16_t sentry2_enable_flag=0;/* 算法使能标记 */
uint8_t sentry2_read_succeed = 0; /* 是否有数据读取成功 */

uint8_t sentry2_enable_succeed_flag = 0;/* 用来检测算法开启是否成功 */
uint8_t sentry2_clear_succeed_flag = 0;/* 用来检测是否指令是否成功 */

/* 数据接收混乱，添加下面标志位增加正确率，判断当前的算法功能是否开启 */
uint8_t sentry2_line_enable = 0;
uint8_t sentry2_blob_enable = 0;

/* 读取的5个数据 */
int sentry2_data1;
int sentry2_data2;
int sentry2_data3;
int sentry2_data4;
int sentry2_data5;

uint8_t sentry2_buf[32];

/* 关闭所有算法 */
void sentry2_clear_all(void)
{
	sentry2_clear_succeed_flag=1;
	if(sentry2_enable_flag==SENTRY2_IS_ENANLE_Blob)
	{
		if(sentry2_enable_succeed_flag==1)/* 要更换算法 */
		{	
			uart2_send_str((u8*)"C4\r\n");
		}
		else
		{
			uart2_send_str((u8*)"C2\r\n");
		}
	}
	else if(sentry2_enable_flag==SENTRY2_IS_ENANLE_Line)
	{
		if(sentry2_enable_succeed_flag==1)/* 要更换算法 */
		{	
			uart2_send_str((u8*)"C2\r\n");
		}
		else
		{
			uart2_send_str((u8*)"C4\r\n");
		}
	}
	else
		sentry2_clear_succeed_flag = 0;
}

/* 开启线条算法 */
void sentry2_start_line_arithmetic(void)
{
	uart2_send_str((u8*)"O4\r\n");
	sentry2_enable_flag = SENTRY2_IS_ENANLE_Line;/* 标记线条算法使能 */ 
	sentry2_enable_succeed_flag = 1;
}

/* 开启物块算法 */
void sentry2_start_blob_arithmetic(void)
{
	uart2_send_str((u8*)"O2\r\n");
	sentry2_enable_flag = SENTRY2_IS_ENANLE_Blob;/* 标记物块算法使能 */ 
	sentry2_enable_succeed_flag=1;
}

/* 读取线条数据 */
void sentry2_read(void)
{
	uart2_send_str((u8*)"A\r\n");
}

/* 获取不同颜色物块的数据 */
void sentry2_read_r(void)
{
	uart2_send_str((u8*)"A1\r\n");
}
void sentry2_read_g(void)
{
	uart2_send_str((u8*)"A2\r\n");
}
void sentry2_read_b(void)
{
	uart2_send_str((u8*)"A3\r\n");
}
