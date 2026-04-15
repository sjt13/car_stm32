// ############################################################
//  FILE:  Task_manager.c
//  Created on: 2021��8��14��
//  Author: lee
//  summary: Task_manager
// ############################################################

#include "main.h"
#include "stdio.h"
#include "usbh_usr.h"

#define Task_Num 5

#define HFPeriod_COUNT 1	// 5ms
#define FaulPeriod_COUNT 10 // 10ms
#define Robot_COUNT 5		// 10ms
#define KEY_COUNT 80		// 10ms
#define LEDPeriod_COUNT 600 // 500ms
#define filter_N 12

extern __ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_Core_dev __ALIGN_END;
extern __ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END;
extern float pitch, roll, yaw;

TaskTime TasksPare[Task_Num];
float dir_anglecmd = 135.0;

void Timer_Task_Count(void) // 1ms���жϼ���
{
	u16 Task_Count = 0;

	for (Task_Count = 0; Task_Count < Task_Num; Task_Count++) // TASK_NUM=5
	{
		if ((TasksPare[Task_Count].Task_Count < TasksPare[Task_Count].Task_Period) && (TasksPare[Task_Count].Task_Period > 0))
		{
			TasksPare[Task_Count].Task_Count++; // ���� �¼��������
		}
	}
}

void Execute_Task_List_RUN(void)
{
	uint16_t Task_Count = 0;

	for (Task_Count = 0; Task_Count < Task_Num; Task_Count++)
	{
		if ((TasksPare[Task_Count].Task_Count >= TasksPare[Task_Count].Task_Period) && (TasksPare[Task_Count].Task_Period > 0))
		{
			TasksPare[Task_Count].Task_Function(); // ���м�����ʱ��������
			TasksPare[Task_Count].Task_Count = 0;
		}
	}
}

void HFPeriod_1msTask(void) // ����
{
	app_sensor_run();
	
	USBH_Process(&USB_OTG_Core_dev, &USB_Host);
	
	app_ps2();

	if (uart1_get_ok) /* ����ָ�� */
	{
		//printf("\r\n  app_uart_run %d= %s \r\n",uart1_get_ok, uart_receive_buf);
		if (uart1_mode == 1)
		{
			uart3_send_str("cmdOk");
			// ����ģʽ��UART6 ���� ELF2 ָ��ʱ��ֻ�����ذ����� $CAR/$DST/$TZ��
			if (uart_receive_num == 6)
			{
				parse_cmd_elf2(uart_receive_buf);
			}
			else
			{
				parse_cmd(uart_receive_buf);
			}
		}
		else if (uart1_mode == 2)
		{
			// �����������
			parse_action(uart_receive_buf);
		}
		else if (uart1_mode == 3)
		{
			// ��·�������
			parse_action(uart_receive_buf);
		}
		else if (uart1_mode == 4)
		{
			// �洢ģʽ
			save_action(uart_receive_buf);
		}
		uart1_mode = 0;
		uart1_get_ok = 0;
		//printf("\r\n  app_uart_run %d= %s \r\n",uart1_get_ok, uart_receive_buf);
	}
	
}

void task_send_Rece(void) 
{
	static u8 tel_div = 0;

	loop_action();

	/*
	 * $TEL 遥测不要在控制环里每次都裸发，不然很容易把串口刷成粥。
	 * 这里 task_send_Rece 是 10ms 一次，5 分频后得到 50ms 一次，也就是 20Hz。
	 * 20Hz 对第一版 odom 调试够用了，先求稳，别装猛。
	 */
	tel_div++;
	if (tel_div >= 5)
	{
		tel_div = 0;
		uart6_report_tel();
	}
}

void Robot_Control(void) // �����˿������񣬺���������
{
	MPU_Get_Gyroscope(imu_gyro_data);
	MPU_Get_Accelerometer(imu_acc_data);
//	printf("mpu gy= %d   %d   %d     ac= %d    %d    %d \r\n", imu_gyro_data[0], imu_gyro_data[1], imu_gyro_data[2],
//		   imu_acc_data[0], imu_acc_data[1], imu_acc_data[2]);
	Robot_Task();
	MPU6050_Update_Angle(&g_mpu6050, 0.01f);
	pitch = MPU6050_Get_Pitch(&g_mpu6050);
	roll  = MPU6050_Get_Roll(&g_mpu6050);
	yaw   = MPU6050_Get_Yaw(&g_mpu6050);
}

void KEY_RUN(void) // �ⲿ������������
{
}

void Task_LED(void) // ��Сϵͳ500ms��LED����˸
{
	GPIO_ToggleBits(GPIOB, GPIO_Pin_6); //  500ms��LED����˸
	
	OLED_ShowString(0, 0, "pitch:", OLED_FONT_6X8);
	OLED_ShowSignedNum(35, 0, (int)pitch, 5, OLED_FONT_6X8);

	OLED_ShowString(0, 1, "roll:", OLED_FONT_6X8);
	OLED_ShowSignedNum(35, 1, (int)roll, 5, OLED_FONT_6X8);

	OLED_ShowString(0, 2, "yaw:", OLED_FONT_6X8);
	OLED_ShowSignedNum(35, 2, (int)yaw, 5, OLED_FONT_6X8);
}

void Task_Manage_List_Init(void)
{
	TasksPare[0].Task_Period = HFPeriod_COUNT; // PERIOD_COUNT=5    5ms
	TasksPare[0].Task_Count = 1;			   // Task_Count�ĳ�ֵ��һ��������500ms��ʱ����������ִ��һ�顣
	TasksPare[0].Task_Function = HFPeriod_1msTask;

	TasksPare[1].Task_Period = FaulPeriod_COUNT; // 10ms
	TasksPare[1].Task_Count = 8;
	TasksPare[1].Task_Function = task_send_Rece; //

	TasksPare[2].Task_Period = Robot_COUNT; // 20ms
	TasksPare[2].Task_Count = 15;
	TasksPare[2].Task_Function = Robot_Control; //

	TasksPare[3].Task_Period = KEY_COUNT; // 100ms
	TasksPare[3].Task_Count = 80;
	TasksPare[3].Task_Function = KEY_RUN; //

	TasksPare[4].Task_Period = LEDPeriod_COUNT; // 500ms
	TasksPare[4].Task_Count = 300;
	TasksPare[4].Task_Function = Task_LED; // 500ms��LED����˸
}
// USER CODE END
