#ifndef _Y_GLOBAL_H_
#define _Y_GLOBAL_H_

#include "main.h"

/*
	宏定义数据
*/
#define VERSION 20230629  // 版本定义
#define ACTION_USE_ROM 0  // 1:动作组使用内部数组动作组	0:动作组使用上位机下载动作组
#define CYCLE 1000		  // PWM模块周期
#define PS2_LED_RED 0x73  // PS2手柄红灯模式
#define PS2_LED_GRN 0x41  // PS2手柄绿灯模式
#define PSX_BUTTON_NUM 16 // 手柄按键数目
#define PS2_MAX_LEN 64	  // 手柄命令最大字节数
#define FLAG_VERIFY 0x25  // 校验标志
#define ACTION_SIZE 256	  // 一个动作的存储大小

#define W25Q64_INFO_ADDR_SAVE_STR (((8 << 10) - 4) << 10) //(8*1024-4)*1024		//eeprom_info结构体存储的位置
#define W25Q64_PS2_ADDR_SAVE_STR (((8 << 10) - 8) << 10)  //(8*1024-4)*1024		//PS2指令存储的位置
#define MODULE "YH-KSTM32"

extern u8 AI_mode;
extern u8 group_do_ok;
extern int do_start_index;	// 动作组执行 起始序号
extern int do_time;			// 动作组执行 执行次数
extern int group_num_start; // 动作组执行 起始序号
extern int group_num_end;	// 动作组执行 终止序号
extern int group_num_times; // 动作组执行 起始变量

extern u8 needSaveFlag;			// 偏差保存标志
extern u32 bias_systick_ms_bak; // 偏差保存标志时间
extern u32 action_time;
extern uint8_t uart_receive_num;

extern u8 forbid_turn;

#define DJ_NUM 8

#define PRE_CMD_SIZE 128
typedef struct
{
	u32 version;
	u32 dj_record_num;
	uint8_t pre_cmd[PRE_CMD_SIZE + 1];
	int dj_bias_pwm[DJ_NUM + 1];
	uint8_t color_base_flag;
	int color_red_base;
	int color_grn_base;
	int color_blu_base;

	uint16_t ps2_cmd_size; /* ps2指令数据大小 */
} eeprom_info_t;

extern uint8_t duoji_index1;
extern uint8_t tim2_stop;

#define CMD_RETURN_SIZE 1024
extern u8 cmd_return[CMD_RETURN_SIZE];
extern eeprom_info_t eeprom_info;

typedef uint8_t u8;
extern u8 uart_receive_buf[1024]; // 接收缓冲,最大UART_BUF_SIZE个字节.末字节为换行符
extern uint16_t uart1_get_ok;	  // 接收完成标记
extern u8 uart1_mode;			  /* 指令的模式 */

uint16_t str_contain_str(unsigned char *str, unsigned char *str2);
int abs_int(int int1);
void selection_sort(int *a, int len);
void replace_char(u8 *str, u8 ch1, u8 ch2);
void int_exchange(int *int1, int *int2);
float abs_float(float value);

void parse_action(u8 *uart_receive_buf);
void parse_cmd(u8 *cmd);
/* UART6 上 ELF2 的 mode-1 白名单闸门（仅允许 $CAR/$DST/$TZ）。 */
void parse_cmd_elf2(u8 *cmd);
void save_action(u8 *str);

void do_group_once(int group_num);
void loop_action(void);
int getMaxTime(u8 *str);
int get_action_index(u8 *str);
void print_group(int start, int end);
void rewrite_eeprom(void);
void set_servo(int index, int pwm, int time);
void zx_uart_send_str(u8 *str);

void soft_reset(void);
void others_init(void);

int kinematics_move(float x, float y, float z, int time);
#endif
