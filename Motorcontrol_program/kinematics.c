#include "main.h"

// ���ݻ��������ͣ�ѡ�����������˶�ѧ��������
#define ROBOT_TYPE ROBOT_MEC

#if (ROBOT_TYPE == ROBOT_MEC)
/**
 * @��  ��  �������˶�ѧ����-�����ķ��
 * @��  ��  ��
 * @����ֵ  ��
 */
void ROBOT_Kinematics(void)
{
	// 先无条件读取一次编码器实时速度，保证停车后 RT_IX/RT_IY/RT_IW 能及时刷新回零。
	// 之前这里一旦目标速度全 0 就直接 return，导致上一次运动时的 RT_* 残值被一直保留，
	// UART6 上行 $TEL 也会反复把旧速度发给 ELF2，最后把 ROS2 /odom 积分带偏。
	Wheel_A.RT = (float)-((int16_t)ENCODER_A_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_A_SetCounter(0);
	Wheel_B.RT = (float)((int16_t)ENCODER_B_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_B_SetCounter(0);
	Wheel_C.RT = (float)-((int16_t)ENCODER_C_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_C_SetCounter(0);
	Wheel_D.RT = (float)((int16_t)ENCODER_D_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_D_SetCounter(0);

	// 用最新编码器值刷新底盘实时速度，哪怕当前目标是停车，也要把实时速度链路更新掉。
	Vel.RT_IX = ((Wheel_A.RT + Wheel_B.RT + Wheel_C.RT + Wheel_D.RT) / 4) * 1000;
	Vel.RT_IY = ((-Wheel_A.RT + Wheel_B.RT + Wheel_C.RT - Wheel_D.RT) / 4) * 1000;
	Vel.RT_IW = ((-Wheel_A.RT + Wheel_B.RT - Wheel_C.RT + Wheel_D.RT) / 4 / (MEC_WHEEL_BASE / 2 + MEC_ACLE_BASE / 2)) * 1000;

	if ((Vel.TG_IX == 0) && (Vel.TG_IY == 0) && (Vel.TG_IW == 0)) /* 目标停车 */
	{
		Vel.TG_FX = 0;
		Vel.TG_FY = 0;
		Vel.TG_FW = 0;
		Wheel_A.TG = 0;
		Wheel_B.TG = 0;
		Wheel_C.TG = 0;
		Wheel_D.TG = 0;
		Wheel_A.PWM = 0;
		Wheel_B.PWM = 0;
		Wheel_C.PWM = 0;
		Wheel_D.PWM = 0;
		MOTOR_A_SetSpeed(0);
		MOTOR_B_SetSpeed(0);
		MOTOR_C_SetSpeed(0);
		MOTOR_D_SetSpeed(0);
		return;
	}

	// ͨ����������ȡ����ʵʱת��m/s

	// 编码器已在函数开头统一读取并清零，这里不再重复做一遍。

	// �����������ת��
	//printf("@%f  %f   %f  %f\r\n",Wheel_A.RT,Wheel_B.RT,Wheel_C.RT,Wheel_D.RT);

	// 底盘实时速度已在函数开头统一刷新，这里直接用最新值继续往下算控制量。

	// ������Ŀ���ٶ�����
	if (Vel.TG_IX > R_VX_LIMIT)
		Vel.TG_IX = R_VX_LIMIT;
	if (Vel.TG_IX < (-R_VX_LIMIT))
		Vel.TG_IX = (-R_VX_LIMIT);
	if (Vel.TG_IY > R_VY_LIMIT)
		Vel.TG_IY = R_VY_LIMIT;
	if (Vel.TG_IY < (-R_VY_LIMIT))
		Vel.TG_IY = (-R_VY_LIMIT);
	if (Vel.TG_IW > R_VW_LIMIT)
		Vel.TG_IW = R_VW_LIMIT;
	if (Vel.TG_IW < (-R_VW_LIMIT))
		Vel.TG_IW = (-R_VW_LIMIT);

	// Ŀ���ٶ�ת��Ϊ��������
	Vel.TG_FX = Vel.TG_IX / 1000.0;
	Vel.TG_FY = Vel.TG_IY / 1000.0;
	Vel.TG_FW = Vel.TG_IW / 1000.0;

	// �˶�ѧ��������ɻ�����Ŀ���ٶȼ����������ٶȣ�m/s��
	Wheel_A.TG = Vel.TG_FX - Vel.TG_FY - Vel.TG_FW * (MEC_WHEEL_BASE / 2 + MEC_ACLE_BASE / 2);
	Wheel_B.TG = Vel.TG_FX + Vel.TG_FY + Vel.TG_FW * (MEC_WHEEL_BASE / 2 + MEC_ACLE_BASE / 2);
	Wheel_C.TG = Vel.TG_FX + Vel.TG_FY - Vel.TG_FW * (MEC_WHEEL_BASE / 2 + MEC_ACLE_BASE / 2);
	Wheel_D.TG = Vel.TG_FX - Vel.TG_FY + Vel.TG_FW * (MEC_WHEEL_BASE / 2 + MEC_ACLE_BASE / 2);

	// printf("@%f  %f  %f  %f \r\n",Wheel_A.TG,Wheel_B.TG,Wheel_C.TG,Wheel_D.TG);

	// ����PID�㷨������PWMֵ
	Wheel_A.PWM = SPEED_PidCtlA(Wheel_A.TG, Wheel_A.RT); // L1
	Wheel_B.PWM = SPEED_PidCtlB(Wheel_B.TG, Wheel_B.RT); // R1
	Wheel_C.PWM = SPEED_PidCtlC(Wheel_C.TG, Wheel_C.RT); // L2
	Wheel_D.PWM = SPEED_PidCtlD(Wheel_D.TG, Wheel_D.RT); // R2

	//printf("@%d  %d  %d  %d \r\n",Wheel_A.PWM,Wheel_B.PWM,Wheel_C.PWM,Wheel_D.PWM);

	// ���õ��PWMֵ
	MOTOR_A_SetSpeed(-Wheel_A.PWM);
	MOTOR_B_SetSpeed(Wheel_B.PWM);
	MOTOR_C_SetSpeed(-Wheel_C.PWM);
	MOTOR_D_SetSpeed(Wheel_D.PWM);
}
#endif

#if (ROBOT_TYPE == ROBOT_FWD)
/**
 * @��  ��  �������˶�ѧ����-���ֲ���
 * @��  ��  ��
 * @����ֵ  ��
 */
void ROBOT_Kinematics(void)
{

	// ͨ����������ȡ����ʵʱת��m/s
	Wheel_A.RT = (float)-((int16_t)ENCODER_A_GetCounter() * FWD_WHEEL_SCALE);
	ENCODER_A_SetCounter(0);
	Wheel_B.RT = (float)((int16_t)ENCODER_B_GetCounter() * FWD_WHEEL_SCALE);
	ENCODER_B_SetCounter(0);
	Wheel_C.RT = (float)-((int16_t)ENCODER_C_GetCounter() * FWD_WHEEL_SCALE);
	ENCODER_C_SetCounter(0);
	Wheel_D.RT = (float)((int16_t)ENCODER_D_GetCounter() * FWD_WHEEL_SCALE);
	ENCODER_D_SetCounter(0);

	// �����������ת��
	// printf("@%f  %f   \r\n",Wheel_A.RT,Wheel_B.RT);

	// �˶�ѧ���������ɻ����������ٶȼ���������ٶ�
	Vel.RT_IX = ((Wheel_A.RT + Wheel_B.RT + Wheel_C.RT + Wheel_D.RT) / 4) * 1000;
	Vel.RT_IY = 0;
	Vel.RT_IW = ((-Wheel_A.RT + Wheel_B.RT - Wheel_C.RT + Wheel_D.RT) / (2 * FWD_WB_SCALE * FWD_WHEEL_BASE)) * 1000;

	// ������Ŀ���ٶ�����
	if (Vel.TG_IX > R_VX_LIMIT)
		Vel.TG_IX = R_VX_LIMIT;
	if (Vel.TG_IX < (-R_VX_LIMIT))
		Vel.TG_IX = (-R_VX_LIMIT);
	if (Vel.TG_IY > R_VY_LIMIT)
		Vel.TG_IY = R_VY_LIMIT;
	if (Vel.TG_IY < (-R_VY_LIMIT))
		Vel.TG_IY = (-R_VY_LIMIT);
	if (Vel.TG_IW > R_VW_LIMIT)
		Vel.TG_IW = R_VW_LIMIT;
	if (Vel.TG_IW < (-R_VW_LIMIT))
		Vel.TG_IW = (-R_VW_LIMIT);

	// Ŀ���ٶ�ת��Ϊ��������
	Vel.TG_FX = Vel.TG_IX / 1000.0;
	Vel.TG_FY = 0;
	Vel.TG_FW = Vel.TG_IW / 1000.0;

	// �˶�ѧ��������ɻ�����Ŀ���ٶȼ����������ٶȣ�m/s��
	Wheel_A.TG = Vel.TG_FX - Vel.TG_FW * (FWD_WHEEL_BASE * FWD_WB_SCALE / 2);
	Wheel_B.TG = Vel.TG_FX + Vel.TG_FW * (FWD_WHEEL_BASE * FWD_WB_SCALE / 2);
	Wheel_C.TG = Vel.TG_FX - Vel.TG_FW * (FWD_WHEEL_BASE * FWD_WB_SCALE / 2);
	Wheel_D.TG = Vel.TG_FX + Vel.TG_FW * (FWD_WHEEL_BASE * FWD_WB_SCALE / 2);

	// ����PID�㷨������PWMֵ
	Wheel_A.PWM = SPEED_PidCtlA(Wheel_A.TG, Wheel_A.RT);
	Wheel_B.PWM = SPEED_PidCtlB(Wheel_B.TG, Wheel_B.RT);
	Wheel_C.PWM = SPEED_PidCtlC(Wheel_C.TG, Wheel_C.RT);
	Wheel_D.PWM = SPEED_PidCtlD(Wheel_D.TG, Wheel_D.RT);

	if (Vel.TG_IX == 0 && Vel.TG_IW == 0)
	{
		Wheel_A.PWM = 0;
		Wheel_B.PWM = 0;
		Wheel_C.PWM = 0;
		Wheel_D.PWM = 0;
	}

	// ���õ��PWMֵ
	MOTOR_A_SetSpeed(Wheel_A.PWM);
	MOTOR_B_SetSpeed(Wheel_B.PWM);
	MOTOR_C_SetSpeed(-Wheel_C.PWM);
	MOTOR_D_SetSpeed(-Wheel_D.PWM);

	// ���������Ϣ
	// printf("@%f  %f  %f %f  \r\n",Wheel_A.RT,Wheel_B.RT, Wheel_C.RT ,Wheel_D.RT);
	// printf("A%d B%d C%d  \r\n ",Vel.I_X, Vel.I_Y, Vel.I_W );
}
#endif

#if (ROBOT_TYPE == ROBOT_AKM)
/**
 * @��  ��  �������˶�ѧ����-������
 * @��  ��  ��
 * @����ֵ  ��
 */
void ROBOT_Kinematics(void)
{

	// ����Ƕ�
	int16_t servo_angle;

	// ͨ����������ȡ����ʵʱת��m/s
	Wheel_A.RT = (float)-((int16_t)ENCODER_A_GetCounter() * AKM_WHEEL_SCALE);
	ENCODER_A_SetCounter(0);
	Wheel_B.RT = (float)((int16_t)ENCODER_B_GetCounter() * AKM_WHEEL_SCALE);
	ENCODER_B_SetCounter(0);

	// �����������ת��
	// printf("@%f  %f   \r\n",Wheel_A.RT,Wheel_B.RT);

	// �˶�ѧ���������ɻ����������ٶȼ���������ٶ�
	Vel.RT_IX = ((Wheel_A.RT + Wheel_B.RT) / 2) * 1000;
	Vel.RT_IY = 0;
	Vel.RT_IW = ((-Wheel_A.RT + Wheel_B.RT) / AKM_WHEEL_BASE) * 1000;

	// ������Ŀ���ٶ�����
	if (Vel.TG_IX > R_VX_LIMIT)
		Vel.TG_IX = R_VX_LIMIT;
	if (Vel.TG_IX < (-R_VX_LIMIT))
		Vel.TG_IX = (-R_VX_LIMIT);
	if (Vel.TG_IY > R_VY_LIMIT)
		Vel.TG_IY = R_VY_LIMIT;
	if (Vel.TG_IY < (-R_VY_LIMIT))
		Vel.TG_IY = (-R_VY_LIMIT);
	if (Vel.TG_IW > R_VW_LIMIT)
		Vel.TG_IW = R_VW_LIMIT;
	if (Vel.TG_IW < (-R_VW_LIMIT))
		Vel.TG_IW = (-R_VW_LIMIT);

	// Ŀ���ٶ�ת��Ϊ��������
	Vel.TG_FX = Vel.TG_IX / 1000.0;
	Vel.TG_FY = Vel.TG_IY / 1000.0;
	Vel.TG_FW = Vel.TG_IW / 1000.0;

	if (Vel.TG_IW != 0)
	{
		// �жϻ�����ǰ���ٶ��Ƿ�Ϊ0
		if (Vel.TG_IX != 0)
		{
			// ����ת��뾶
			RobotStr.Radius = Vel.TG_FX / Vel.TG_FW;

			// ��������������Ҫ������Сת��뾶
			// ���Ŀ���ٶ�Ҫ���ת��뾶С����Сת��뾶��
			// �ᵼ�»������˶�Ħ���������ߣ�����Ӱ�����Ч��

			// ת��뾶С����Сת��
			if (RobotStr.Radius > 0 && RobotStr.Radius < AKM_TURN_R_MINI)
			{
				RobotStr.Radius = AKM_TURN_R_MINI;
			}

			else if (RobotStr.Radius < 0 && RobotStr.Radius > (-AKM_TURN_R_MINI))
			{
				RobotStr.Radius = -AKM_TURN_R_MINI;
			}

			// ���������ǰ��ת��Ƕ�,��λ����
			RobotStr.Angle = atan(AKM_ACLE_BASE / (RobotStr.Radius));

			// �˶�ѧ��������ɻ�����Ŀ���ٶȼ����������ٶȣ�m/s��
			Wheel_A.TG = Vel.TG_FX * (RobotStr.Radius - 0.5 * AKM_WHEEL_BASE) / RobotStr.Radius;
			Wheel_B.TG = Vel.TG_FX * (RobotStr.Radius + 0.5 * AKM_WHEEL_BASE) / RobotStr.Radius;
		}
		else
		{
			Wheel_A.TG = 0;
			Wheel_B.TG = 0;
			RobotStr.Radius = 0;
			RobotStr.Angle = 0;
		}
	}
	else
	{
		Wheel_A.TG = Vel.TG_FX;
		Wheel_B.TG = Vel.TG_FX;
		RobotStr.Radius = 0;
		RobotStr.Angle = 0;
	}

	// ����ǰ�ֽǶȼ�����ǰ�ֽǶ�
	if (RobotStr.Angle != 0)
	{
		RobotStr.RAngle = (atan(AKM_ACLE_BASE / ((AKM_ACLE_BASE / tan(RobotStr.Angle * 0.01745)) + 0.5 * AKM_WHEEL_BASE))) * (180.0 / PI);
	}
	else
	{
		RobotStr.RAngle = 0;
	}

	// ������ǰ�ֽǶȣ�������ת��Ƕ�
	RobotStr.SAngle = -(0.0041 * RobotStr.RAngle * RobotStr.RAngle + 1.2053 * RobotStr.RAngle) * 180 / PI;

	// ���ݶ��ת��Ƕȣ�������PWM������
	servo_angle = (RobotStr.SAngle * 10 + 900 + ax_servo_offset);

	// ����PID�㷨������PWMֵ
	Wheel_A.PWM = SPEED_PidCtlA(Wheel_A.TG, Wheel_A.RT);
	Wheel_B.PWM = SPEED_PidCtlB(Wheel_B.TG, Wheel_B.RT);

	// ���õ��PWMֵ
	MOTOR_A_SetSpeed(Wheel_A.PWM);
	MOTOR_B_SetSpeed(Wheel_B.PWM);

	// ���ö���Ƕȣ�S1,S2����һ��������
	SERVO_S1_SetAngle(servo_angle);
	SERVO_S2_SetAngle(servo_angle);
}
#endif

#if (ROBOT_TYPE == ROBOT_TWD)
/**
 * @��  ��  �������˶�ѧ����-���ֲ���
 * @��  ��  ��
 * @����ֵ  ��
 */
void ROBOT_Kinematics(void)
{

	// ͨ����������ȡ����ʵʱת��m/s
	Wheel_A.RT = (float)-((int16_t)ENCODER_A_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_A_SetCounter(0);
	Wheel_B.RT = (float)((int16_t)ENCODER_B_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_B_SetCounter(0);

	// �����������ת��
	// printf("@%f  %f   \r\n",Wheel_A.RT,Wheel_B.RT);

	// �˶�ѧ���������ɻ����������ٶȼ���������ٶ�
	Vel.RT_IX = ((Wheel_A.RT + Wheel_B.RT) / 2) * 1000;
	Vel.RT_IY = 0;
	Vel.RT_IW = ((-Wheel_A.RT + Wheel_B.RT) / TWD_WHEEL_BASE) * 1000;

	// ������Ŀ���ٶ�����
	if (Vel.TG_IX > R_VX_LIMIT)
		Vel.TG_IX = R_VX_LIMIT;
	if (Vel.TG_IX < (-R_VX_LIMIT))
		Vel.TG_IX = (-R_VX_LIMIT);
	if (Vel.TG_IY > R_VY_LIMIT)
		Vel.TG_IY = R_VY_LIMIT;
	if (Vel.TG_IY < (-R_VY_LIMIT))
		Vel.TG_IY = (-R_VY_LIMIT);
	if (Vel.TG_IW > R_VW_LIMIT)
		Vel.TG_IW = R_VW_LIMIT;
	if (Vel.TG_IW < (-R_VW_LIMIT))
		Vel.TG_IW = (-R_VW_LIMIT);

	// Ŀ���ٶ�ת��Ϊ��������
	Vel.TG_FX = Vel.TG_IX / 1000.0;
	Vel.TG_FY = 0;
	Vel.TG_FW = Vel.TG_IW / 1000.0;

	// �˶�ѧ��������ɻ�����Ŀ���ٶȼ����������ٶȣ�m/s��
	Wheel_A.TG = Vel.TG_FX - Vel.TG_FW * (TWD_WHEEL_BASE / 2);
	Wheel_B.TG = Vel.TG_FX + Vel.TG_FW * (TWD_WHEEL_BASE / 2);

	// ����PID�㷨������PWMֵ
	Wheel_A.PWM = SPEED_PidCtlA(Wheel_A.TG, Wheel_A.RT);
	Wheel_B.PWM = SPEED_PidCtlB(Wheel_B.TG, Wheel_B.RT);

	// ���õ��PWMֵ
	MOTOR_A_SetSpeed(Wheel_A.PWM);
	MOTOR_B_SetSpeed(Wheel_B.PWM);

	// printf("A%f B%f  \r\n ",MOTOR_A.Wheel_RT, MOTOR_B.Wheel_RT  );
	// printf("A%d B%d C%d  \r\n ",Vel.I_X, Vel.I_Y, Vel.I_W );
}
#endif

/**
 * @��  ��  ������ֹͣ�˶�״̬
 * @��  ��  ��
 * @����ֵ  ��
 */
void ROBOT_Stop(void)
{

	// ���õ��PWMֵ
	MOTOR_A_SetSpeed(0);
	MOTOR_B_SetSpeed(0);
	MOTOR_C_SetSpeed(0);
	MOTOR_D_SetSpeed(0);
}

#define pi 3.1415926f

kinematics_t kinematics;

/*
	�����ĸ��ؽڵĳ���
	��λ1mm
*/

void setup_kinematics(float L0, float L1, float L2, float L3, kinematics_t *kinematics) {
	//�Ŵ�10��
	kinematics->L0 = L0*10;
	kinematics->L1 = L1*10;
	kinematics->L2 = L2*10;
	kinematics->L3 = L3*10;
}

/*
	x,y Ϊӳ�䵽ƽ�������
	zΪ�������ľ���
	Alpha Ϊצ�Ӻ�ƽ��ļн� -25~-65��Χ�ȽϺ�
*/

int kinematics_analysis(float x, float y, float z, float Alpha, kinematics_t *kinematics) {
	float theta3, theta4, theta5, theta6;
	float l0, l1, l2, l3;
	float aaa, bbb, ccc, zf_flag;
	
	//�Ŵ�10��
	x = x*10;
	y = y*10;
	z = z*10;
	
	
	l0 = kinematics->L0;
	l1 = kinematics->L1;
	l2 = kinematics->L2;
	l3 = kinematics->L3;
	
	if(x == 0) {
		theta6 = 0.0;
	} else {
		theta6 = atan(x/y)*270.0f/pi;
	}
	
	y = sqrt(x*x + y*y);    
    y = y-l3 * cos(Alpha*pi/180.0f);  
    z = z-l0-l3*sin(Alpha*pi/180.0f); 
    if(z < -l0) {
        return 1;
	}
    if(sqrt(y*y + z*z) > (l1+l2)) {
        return 2;
	}
	
	ccc = acos(y / sqrt(y * y + z * z));
    bbb = (y*y+z*z+l1*l1-l2*l2)/(2*l1*sqrt(y*y+z*z));
    if(bbb > 1 || bbb < -1) {
        return 5;
	}
    if (z < 0) {
        zf_flag = -1;
	} else {
        zf_flag = 1;
	}
    theta5 = ccc * zf_flag + acos(bbb);
    theta5 = theta5 * 180.0f / pi;
    if(theta5 > 180.0f || theta5 < 0.0f) {
        return 6;
	}
	
    aaa = -(y*y+z*z-l1*l1-l2*l2)/(2*l1*l2);
    if (aaa > 1 || aaa < -1) {
        return 3;
	}
    theta4 = acos(aaa); 
    theta4 = 180.0f - theta4 * 180.0f / pi ;  
    if (theta4 > 135.0f || theta4 < -135.0f) {
        return 4;
	}

    theta3 = Alpha - theta5 + theta4;
    if(theta3 > 90.0f || theta3 < -90.0f) {
        return 7;
	}
	
	kinematics->servo_angle[0] = theta6;
	kinematics->servo_angle[1] = theta5-90;
	kinematics->servo_angle[2] = theta4;
	kinematics->servo_angle[3] = theta3;    
	
	kinematics->servo_pwm[0] = (int)(1500-2000.0f * kinematics->servo_angle[0] / 270.0f);
	kinematics->servo_pwm[1] = (int)(1500+2000.0f * kinematics->servo_angle[1] / 270.0f);
	kinematics->servo_pwm[2] = (int)(1500+2000.0f * kinematics->servo_angle[2] / 270.0f);
	kinematics->servo_pwm[3] = (int)(1500-2000.0f * kinematics->servo_angle[3] / 270.0f);

	return 0;
}
