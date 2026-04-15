// ############################################################
//  文件名: Task_manager.c
//  创建时间: 2021年8月14日
//  作者: lee
//  功能: 任务调度与周期任务管理
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

void Timer_Task_Count(void) // 1ms的中断计数
{
	u16 Task_Count = 0;

	for (Task_Count = 0; Task_Count < Task_Num; Task_Count++) // 任务总数=5
	{
		if ((TasksPare[Task_Count].Task_Count < TasksPare[Task_Count].Task_Period) && (TasksPare[Task_Count].Task_Period > 0))
		{
			TasksPare[Task_Count].Task_Count++; // 计数 事件任务计数
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
			TasksPare[Task_Count].Task_Function(); // 运行计数的时间任务函数
			TasksPare[Task_Count].Task_Count = 0;
		}
	}
}

void HFPeriod_1msTask(void) // 备用
{
	app_sensor_run();
	
	USBH_Process(&USB_OTG_Core_dev, &USB_Host);
	
	app_ps2();

	if (uart1_get_ok) /* 接收指令 */
	{
		//printf("\r\n  app_uart_run %d= %s \r\n",uart1_get_ok, uart_receive_buf);
		if (uart1_mode == 1)
		{
			uart3_send_str("cmdOk");
			// 命令模式
			/*
			 * 仅当命令来源是 UART6 且模式为 mode-1（$...!）时，
			 * 才进入 parse_cmd_elf2() 进行 ELF2 白名单过滤：
			 * 允许 $CAR / $DST / $TZ，其他 $ 命令直接忽略。
			 * 其余串口来源保持 parse_cmd() 全量能力，不受影响。
			 */
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
			// 单个舵机调试
			parse_action(uart_receive_buf);
		}
		else if (uart1_mode == 3)
		{
			// 多路舵机调试
			parse_action(uart_receive_buf);
		}
		else if (uart1_mode == 4)
		{
			// 存储模式
			save_action(uart_receive_buf);
		}
		uart1_mode = 0;
		uart1_get_ok = 0;
		//printf("\r\n  app_uart_run %d= %s \r\n",uart1_get_ok, uart_receive_buf);
	}
	
}

/*
 * 组装并发送一帧 ELF2 遥测数据（仅从 UART6 发出）。
 * 帧格式固定为：
 * $TEL:ix,iy,iw,wa,wb,wc,wd,ax,ay,az,gx,gy,gz,pit,rol,yaw!
 * 字段顺序固定，不可调整，否则上位机解析会错位。
 */
static void elf2_telemetry_task(void)
{
	char telemetry_buf[196];
	int len;
	/* 车轮实时速度 Wheel_*.RT 单位为 m/s；乘 1000 后按整数上报。 */
	/* 姿态角 pitch/roll/yaw 乘 100 后转整数，保留两位小数精度。 */
	int wa = (int)(Wheel_A.RT * 1000.0);
	int wb = (int)(Wheel_B.RT * 1000.0);
	int wc = (int)(Wheel_C.RT * 1000.0);
	int wd = (int)(Wheel_D.RT * 1000.0);
	int pit = (int)(pitch * 100.0f);
	int rol = (int)(roll * 100.0f);
	int yw = (int)(yaw * 100.0f);

	/*
	 * 字段顺序保持为：
	 * 底盘层编码器 -> 车轮层编码器 -> 加速度 -> 陀螺仪 -> 姿态角
	 * 上位机依赖“字段总数 + 字段顺序”进行固定位置解析。
	 */
	len = snprintf(telemetry_buf, sizeof(telemetry_buf),
					"$TEL:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d!",
					Vel.RT_IX, Vel.RT_IY, Vel.RT_IW,
					wa, wb, wc, wd,
					imu_acc_data[0], imu_acc_data[1], imu_acc_data[2],
					imu_gyro_data[0], imu_gyro_data[1], imu_gyro_data[2],
					pit, rol, yw);

	/* 仅在 snprintf 成功且未截断时发送，避免输出半帧或越界数据。 */
	if (len > 0 && len < (int)sizeof(telemetry_buf))
	{
		uart6_send_str((u8 *)telemetry_buf);
	}
}

void task_send_Rece(void)
{
	static u8 telemetry_div = 0;

	loop_action();

	/* task_send_Rece 每 10ms 执行一次；2 分频后即 20ms，等效 50Hz。 */
	telemetry_div++;
	if (telemetry_div >= 2)
	{
		telemetry_div = 0;
		elf2_telemetry_task();
	}
}


void Robot_Control(void) // 机器人控制任务，含电量管理
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

void KEY_RUN(void) // 外部按键处理任务
{
}

void Task_LED(void) // 最小系统500ms的LED的闪烁
{
	GPIO_ToggleBits(GPIOB, GPIO_Pin_6); //  500ms的LED的闪烁
	
	OLED_ShowString(0, 0, "pitch:", OLED_FONT_6X8);
	OLED_ShowSignedNum(35, 0, (int)pitch, 5, OLED_FONT_6X8);

	OLED_ShowString(0, 1, "roll:", OLED_FONT_6X8);
	OLED_ShowSignedNum(35, 1, (int)roll, 5, OLED_FONT_6X8);

	OLED_ShowString(0, 2, "yaw:", OLED_FONT_6X8);
	OLED_ShowSignedNum(35, 2, (int)yaw, 5, OLED_FONT_6X8);
}

void Task_Manage_List_Init(void)
{
	TasksPare[0].Task_Period = HFPeriod_COUNT; // 周期计数=5ms
	TasksPare[0].Task_Count = 1;			   // Task_Count的初值不一样，避免500ms到时，所有任务都执行一遍。
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
	TasksPare[4].Task_Function = Task_LED; // 500ms的LED的闪烁
}
// 用户代码结束
