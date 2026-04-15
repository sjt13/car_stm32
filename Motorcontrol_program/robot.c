#include "main.h"

// 机器人速度数据
ROBOT_Velocity Vel;

// 机器人轮子数据
ROBOT_Wheel Wheel_A, Wheel_B, Wheel_C, Wheel_D;

// 机器人RGB数据
ROBOT_Light Light;

// 机器人电池电压数据
uint16_t Bat_Vol;

// IMU数据
int16_t imu_acc_data[3];
int16_t imu_gyro_data[3];
int16_t imu_gyro_offset[3];

// 电机PID控制参数
int16_t motor_kp = 800;
int16_t motor_kd = 400;


// 阿克曼机器人专用，转向数据
ROBOT_Steering RobotStr;

// 阿克曼机器人专用，转向数据
int16_t servo_offset = 0;

blance_samp blance_sampPara = blance_samp_DEFAULTS;

/**
 * @简  述  机器人管理任务
 * @参  数  无
 * @返回值  无
 */
void Robot_Task(void)
{
	// 机器人运动学处理
	ROBOT_Kinematics();
}

/**
 * @简  述  电量管理任务
 * @参  数  无
 * @返回值  无
 */
void Bat_Task()
{
	// 计数变量
	static uint16_t bat_vol_cnt = 0;

	while (1)
	{
		// 采集电池电压
		Bat_Vol = VIN_GetVol_X100();

		// 调试输出电池电压数据
		// printf("@ %d  \r\n",R_Bat_Vol);

		// 电量低于40%
		if (Bat_Vol < VBAT_40P)
		{
			// 电量低于20%
			if (Bat_Vol < VBAT_20P)
			{

				// 电量低于10%，关闭系统进入保护状态
				if (Bat_Vol < VBAT_10P) // 990
				{
					// 低压时间计数
					bat_vol_cnt++;

					// 超过10次，进入关闭状态
					if (bat_vol_cnt > 10)
					{
						// 电机速度设置为0
						MOTOR_A_SetSpeed(0);
						MOTOR_B_SetSpeed(0);
						MOTOR_C_SetSpeed(0);
						MOTOR_D_SetSpeed(0);

						// 蜂鸣器鸣叫报警
						while (1)
						{
							BEEP_On();
							BEEP_Off();
						}
					}
				}
				else
				{
					bat_vol_cnt = 0;
				}
			}
		}
		else
		{
		}
	}
}

/**
 * @简  述  按键处理任务
 * @参  数  无
 * @返回值  无
 */
void Key_Task()
{
	while (1)
	{
		// 按键扫描
		if (KEY_Scan() == 0)
		{
			Delay_ms(5);
			// 确定按键按下
			if (KEY_Scan() == 0)
			{
				// 等待按键抬起
				while (KEY_Scan() == 0)
				{
					Delay_ms(5);
				}
			}
		}
	}
}

