//############################################################
// FILE: GPIO_int.c
// Created on: 2017年1月18日
// Author: lee
// summary: GPIO_int
//############################################################

#ifndef _GPIO_int_H
#define _GPIO_int_H
#include "Sys.h"    

/* 循迹引脚读取，从右数1-》4 */

#define TRTACK_IR4_X1_PIN GPIO_Pin_4
#define TRTACK_IR4_X1_PORT GPIOC               /* GPIO端口 */
#define TRTACK_IR4_X1_CLK RCC_AHB1Periph_GPIOC /* GPIO端口时钟 */

#define TRTACK_IR4_X2_PIN GPIO_Pin_5
#define TRTACK_IR4_X2_PORT GPIOC               /* GPIO端口 */
#define TRTACK_IR4_X2_CLK RCC_AHB1Periph_GPIOC /* GPIO端口时钟 */

#define TRTACK_IR4_X3_PIN GPIO_Pin_0
#define TRTACK_IR4_X3_PORT GPIOB               /* GPIO端口 */
#define TRTACK_IR4_X3_CLK RCC_AHB1Periph_GPIOB /* GPIO端口时钟 */

#define TRTACK_IR4_X4_PIN GPIO_Pin_1
#define TRTACK_IR4_X4_PORT GPIOB               /* GPIO端口 */
#define TRTACK_IR4_X4_CLK RCC_AHB1Periph_GPIOB /* GPIO端口时钟 */

#define TRTACK_IR4_X1_READ() GPIO_ReadInputDataBit(TRTACK_IR4_X1_PORT,TRTACK_IR4_X1_PIN)
#define TRTACK_IR4_X2_READ() GPIO_ReadInputDataBit(TRTACK_IR4_X2_PORT,TRTACK_IR4_X2_PIN)
#define TRTACK_IR4_X3_READ() GPIO_ReadInputDataBit(TRTACK_IR4_X3_PORT,TRTACK_IR4_X3_PIN)
#define TRTACK_IR4_X4_READ() GPIO_ReadInputDataBit(TRTACK_IR4_X4_PORT,TRTACK_IR4_X4_PIN)
 
#define TRIG_SET(x) GPIO_WriteBit(GPIOA, GPIO_Pin_3, (BitAction)x)
#define ECHO_GET() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)

//#define LED2_REV   GPIO_ToggleBits(GPIOB,GPIO_Pin_6)
void MOTOR_AB_Init(void); //电机PWM控制初始化
void MOTOR_CD_Init(void); //电机PWM控制初始化
void GPIO_LED_int(void);
void MOTOR_AB_Init(void);
void MOTOR_CD_Init(void);
void ENCODER_A_Init(void);
uint16_t ENCODER_A_GetCounter(void);
void ENCODER_A_SetCounter(uint16_t count);
void ENCODER_B_Init(void);
uint16_t ENCODER_B_GetCounter(void);
void ENCODER_B_SetCounter(uint16_t count);
void ENCODER_C_Init(void);
uint16_t ENCODER_C_GetCounter(void);
void ENCODER_C_SetCounter(uint16_t count);
void ENCODER_D_Init(void);
uint16_t ENCODER_D_GetCounter(void);
void ENCODER_D_SetCounter(uint16_t count);
void KEY_Init(void);
void VIN_Init(void);
uint16_t VIN_GetVol_X100(void);

uint32_t TIM_IsActiveFlag_UPDATE(TIM_TypeDef *TIMx);
uint32_t TIM_IsEnabledIT_UPDATE(TIM_TypeDef *TIMx);
uint32_t TIM_GetDirection(TIM_TypeDef *TIMx);

void TRACK_IR4_Init(void);

void ultrasonic_sensor_init(void);
float sensor_sr_ultrasonic_read(void);
#endif   //  GPIO_int.h
//===========================================================================
// No more.
//===========================================================================
