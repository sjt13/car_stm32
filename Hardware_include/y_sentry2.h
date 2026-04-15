#ifndef _SENTRY2_H_
#define _SENTRY2_H_

#include "main.h"


#define SENTRY2_IS_ENANLE_Blob 0X02
#define SENTRY2_IS_ENANLE_Line 0X08

extern uint16_t sentry2_enable_flag;/* 算法使能标记 */
extern uint8_t sentry2_read_succeed;/* 是否有数据读取成功 */
extern uint8_t sentry2_clear_succeed_flag;
extern uint8_t sentry2_enable_succeed_flag;

/* 数据接收混乱，添加下面标志位增加正确率，判断当前的算法功能是否开启 */
extern uint8_t sentry2_line_enable;
extern uint8_t sentry2_blob_enable;

/* 读取的5个数据 */
extern int sentry2_data1;
extern int sentry2_data2;
extern int sentry2_data3;
extern int sentry2_data4;
extern int sentry2_data5;

extern uint8_t sentry2_buf[32];

void sentry2_clear_all(void);

void sentry2_start_line_arithmetic(void);
void sentry2_start_blob_arithmetic(void);
void sentry2_read(void);

void sentry2_read_r(void);
void sentry2_read_g(void);
void sentry2_read_b(void);

#endif



