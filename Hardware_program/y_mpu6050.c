#include "y_mpu6050.h"

/*----------------------------------------------------------
 * 全局姿态对象
 *---------------------------------------------------------*/
MPU6050_Attitude_t g_mpu6050;

/*----------------------------------------------------------
 * 简单延时函数
 * 注意：这是粗略延时，精度依赖主频和编译优化
 * 若项目中已有更精确的 delay_us / HAL_Delay，可替换
 *---------------------------------------------------------*/
static void Delay2Us(void)
{
    u8 i;
    for (i = 0; i < 12; i++)
    {
        __nop();
    }
}

static void DelayMs_1(void)
{
    u16 i;
    for (i = 0; i < 200; i++)
    {
        Delay2Us();
    }
}

static void Delayms(u16 n)
{
    u16 i;
    for (i = 0; i < n; i++)
    {
        DelayMs_1();
    }
}

/*----------------------------------------------------------
 * 初始化MPU6050
 * 返回值：
 *  0 -> 成功
 *  1 -> 失败（设备ID不对或通信异常）
 *---------------------------------------------------------*/
u8 MPU_Init(void)
{
    u8 res;

    IIC_Init();              // 初始化软件IIC
    Delayms(100);

    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80);   // 复位MPU6050
    Delayms(100);

    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00);   // 唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);                       // 陀螺仪量程：±2000dps
    MPU_Set_Accel_Fsr(0);                      // 加速度量程：±2g
    MPU_Set_Rate(100);                         // 采样率：100Hz，便于姿态解算

    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);      // 关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00);   // 关闭I2C主模式
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);     // 关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80);   // INT引脚低电平有效

    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);    // 读取设备ID，正常应为0x68

    if (res == MPU_ADDR)
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01);  // 时钟源：PLL with X gyro
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00);  // 加速度和陀螺仪全部使能
        MPU_Set_Rate(100);

        memset(&g_mpu6050, 0, sizeof(g_mpu6050));
        return 0;
    }

    return 1;
}

/*----------------------------------------------------------
 * 设置陀螺仪满量程
 * fsr:
 *  0 -> ±250dps
 *  1 -> ±500dps
 *  2 -> ±1000dps
 *  3 -> ±2000dps
 *---------------------------------------------------------*/
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);
}

/*----------------------------------------------------------
 * 设置加速度计满量程
 * fsr:
 *  0 -> ±2g
 *  1 -> ±4g
 *  2 -> ±8g
 *  3 -> ±16g
 *---------------------------------------------------------*/
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);
}

/*----------------------------------------------------------
 * 设置数字低通滤波器 LPF
 * lpf: 截止频率(Hz)
 *---------------------------------------------------------*/
u8 MPU_Set_LPF(u16 lpf)
{
    u8 data = 0;

    if (lpf >= 188)      data = 1;
    else if (lpf >= 98)  data = 2;
    else if (lpf >= 42)  data = 3;
    else if (lpf >= 20)  data = 4;
    else if (lpf >= 10)  data = 5;
    else                 data = 6;

    return MPU_Write_Byte(MPU_CFG_REG, data);
}

/*----------------------------------------------------------
 * 设置采样率
 * rate 范围：4~1000Hz
 * 内部采样率基准按 1KHz 计算
 *---------------------------------------------------------*/
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;

    if (rate > 1000) rate = 1000;
    if (rate < 4)    rate = 4;

    data = 1000 / rate - 1;
    MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);

    return MPU_Set_LPF(rate / 2);  // 一般将LPF设置为采样率一半
}

/*----------------------------------------------------------
 * 获取温度
 * 返回值：温度值*100
 * 例如 3653 表示 36.53℃
 *---------------------------------------------------------*/
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;

    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((short)buf[0] << 8) | buf[1];
    temp = 36.53f + ((float)raw) / 340.0f;

    return (short)(temp * 100.0f);
}

/*----------------------------------------------------------
 * 获取陀螺仪原始值
 * pbuf[0] -> gx_raw
 * pbuf[1] -> gy_raw
 * pbuf[2] -> gz_raw
 *---------------------------------------------------------*/
void MPU_Get_Gyroscope(int16_t *pbuf)
{
    uint8_t buf[6];

    MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);

    pbuf[0] = ((int16_t)buf[0] << 8) | buf[1];
    pbuf[1] = ((int16_t)buf[2] << 8) | buf[3];
    pbuf[2] = ((int16_t)buf[4] << 8) | buf[5];
}

/*----------------------------------------------------------
 * 获取加速度原始值
 * pbuf[0] -> ax_raw
 * pbuf[1] -> ay_raw
 * pbuf[2] -> az_raw
 *---------------------------------------------------------*/
void MPU_Get_Accelerometer(int16_t *pbuf)
{
    uint8_t buf[6];

    MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);

    pbuf[0] = ((int16_t)buf[0] << 8) | buf[1];
    pbuf[1] = ((int16_t)buf[2] << 8) | buf[3];
    pbuf[2] = ((int16_t)buf[4] << 8) | buf[5];
}

/*----------------------------------------------------------
 * 连续写
 *---------------------------------------------------------*/
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;

    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg);
    IIC_Wait_Ack();

    for (i = 0; i < len; i++)
    {
        IIC_Send_Byte(buf[i]);
        if (IIC_Wait_Ack())
        {
            IIC_Stop();
            return 1;
        }
    }

    IIC_Stop();
    return 0;
}

/*----------------------------------------------------------
 * 连续读
 *---------------------------------------------------------*/
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg);
    IIC_Wait_Ack();

    IIC_Start();
    IIC_Send_Byte((addr << 1) | 1);
    IIC_Wait_Ack();

    while (len)
    {
        if (len == 1)
            *buf = IIC_Read_Byte(0);   // 最后一个字节发送NACK
        else
            *buf = IIC_Read_Byte(1);   // 其余发送ACK

        len--;
        buf++;
    }

    IIC_Stop();
    return 0;
}

/*----------------------------------------------------------
 * 写一个字节
 *---------------------------------------------------------*/
u8 MPU_Write_Byte(u8 reg, u8 data)
{
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg);
    IIC_Wait_Ack();

    IIC_Send_Byte(data);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Stop();
    return 0;
}

/*----------------------------------------------------------
 * 读一个字节
 *---------------------------------------------------------*/
u8 MPU_Read_Byte(u8 reg)
{
    u8 res;

    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0);
    IIC_Wait_Ack();

    IIC_Send_Byte(reg);
    IIC_Wait_Ack();

    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 1);
    IIC_Wait_Ack();

    res = IIC_Read_Byte(0);
    IIC_Stop();

    return res;
}

/*----------------------------------------------------------
 * 设置FIFO
 * 这个函数你原文件里声明了但没有定义，这里补上
 * sens:
 *  bit6 TEMP_OUT
 *  bit5 XG
 *  bit4 YG
 *  bit3 ZG
 *  bit2 ACCEL
 *---------------------------------------------------------*/
u8 MPU_Set_Fifo(u8 sens)
{
    return MPU_Write_Byte(MPU_FIFO_EN_REG, sens);
}

/*----------------------------------------------------------
 * 读取全部原始数据并换算为物理量
 *
 * 注意：
 * 1. 当前默认配置：
 *    - 加速度 ±2g
 *    - 陀螺仪 ±2000dps
 * 2. 如果你后面改了量程，这里的灵敏度也要对应修改
 *---------------------------------------------------------*/
void MPU6050_Read_All(MPU6050_Attitude_t *mpu)
{
    MPU_Get_Accelerometer(mpu->acc_raw);
    MPU_Get_Gyroscope(mpu->gyro_raw);

    /* 原始值 -> g */
    mpu->ax = (float)mpu->acc_raw[0] / MPU_ACCEL_SENS_2G;
    mpu->ay = (float)mpu->acc_raw[1] / MPU_ACCEL_SENS_2G;
    mpu->az = (float)mpu->acc_raw[2] / MPU_ACCEL_SENS_2G;

    /* 原始值 -> °/s */
    mpu->gx = (float)mpu->gyro_raw[0] / MPU_GYRO_SENS_2000DPS - mpu->gyro_offset_x;
    mpu->gy = (float)mpu->gyro_raw[1] / MPU_GYRO_SENS_2000DPS - mpu->gyro_offset_y;
    mpu->gz = (float)mpu->gyro_raw[2] / MPU_GYRO_SENS_2000DPS - mpu->gyro_offset_z;
}

/*----------------------------------------------------------
 * 陀螺仪零偏校准
 *
 * 原理：
 * 模块静止时，理论上角速度应为 0
 * 实际存在零漂，因此对静止状态采样多次求平均，作为偏置
 *
 * 使用要求：
 * - 校准期间模块必须保持静止
 * - 建议上电后先延时一小段时间再校准
 *---------------------------------------------------------*/
void MPU6050_Gyro_Calibration(MPU6050_Attitude_t *mpu, uint16_t times)
{
    uint16_t i;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    int16_t gyro[3];

    if (times == 0)
        times = 1000;

    for (i = 0; i < times; i++)
    {
        MPU_Get_Gyroscope(gyro);

        sum_x += gyro[0];
        sum_y += gyro[1];
        sum_z += gyro[2];

        Delayms(2); // 简单间隔，避免读太快
    }

    mpu->gyro_offset_x = ((float)sum_x / times) / MPU_GYRO_SENS_2000DPS;
    mpu->gyro_offset_y = ((float)sum_y / times) / MPU_GYRO_SENS_2000DPS;
    mpu->gyro_offset_z = ((float)sum_z / times) / MPU_GYRO_SENS_2000DPS;
}

/*----------------------------------------------------------
 * 初始化姿态角
 *
 * 原理：
 * 上电初始时，Yaw 无法从加速度计直接求出，先置零
 * Pitch/Roll 可由重力方向估算
 *
 * pitch = atan2( ay, sqrt(ax^2 + az^2) ) * 180/pi
 * roll  = atan2(-ax, az)                 * 180/pi
 *
 * 说明：
 * 不同安装方向下，公式可能需要调整符号
 * 若显示方向和实际相反，只需交换符号或坐标轴
 *---------------------------------------------------------*/
void MPU6050_Angle_Init(MPU6050_Attitude_t *mpu)
{
    float acc_pitch, acc_roll;

    MPU6050_Read_All(mpu);

    acc_pitch = atan2f(mpu->ay, sqrtf(mpu->ax * mpu->ax + mpu->az * mpu->az)) * RAD_TO_DEG;
    acc_roll  = atan2f(-mpu->ax, mpu->az) * RAD_TO_DEG;

    mpu->pitch = acc_pitch;
    mpu->roll  = acc_roll;
    mpu->yaw   = 0.0f;      // 无磁力计，偏航角初值只能手动设定/默认0
    mpu->angle_inited = 1;
}

/*----------------------------------------------------------
 * 姿态更新：互补滤波
 *
 * 输入：
 *  dt -> 两次更新间隔（单位：秒）
 *
 * 流程：
 * 1. 读取加速度和陀螺仪
 * 2. 用加速度算出 Pitch、Roll 的“绝对参考角”
 * 3. 用陀螺仪对上一时刻角度积分，得到短时变化
 * 4. 用互补滤波融合两者
 *
 * 数学思想：
 *  angle = alpha * (angle + gyro * dt) + (1-alpha) * acc_angle
 *
 * 特点：
 * - 陀螺仪：短时间平滑、响应快，但会漂移
 * - 加速度计：长期稳定，但抖动较大、易受线性运动影响
 * - 互补滤波：把两者优点结合
 *
 * 偏航角说明：
 * - yaw 只能由 gz 积分得到
 * - 长时间一定会漂移
 *---------------------------------------------------------*/
void MPU6050_Update_Angle(MPU6050_Attitude_t *mpu, float dt)
{
    float acc_pitch, acc_roll;
    float gyro_pitch, gyro_roll, gyro_yaw;
    const float alpha = MPU_COMPL_FILTER_ALPHA;

    if (dt <= 0.0f)
        return;

    MPU6050_Read_All(mpu);

    /*---------- 1) 由加速度计算俯仰角和横滚角 ----------*/
    acc_pitch = atan2f(mpu->ay, sqrtf(mpu->ax * mpu->ax + mpu->az * mpu->az)) * RAD_TO_DEG;
    acc_roll  = atan2f(-mpu->ax, mpu->az) * RAD_TO_DEG;

    /* 若尚未初始化，则直接赋初值 */
    if (mpu->angle_inited == 0)
    {
        mpu->pitch = acc_pitch;
        mpu->roll  = acc_roll;
        mpu->yaw   = 0.0f;
        mpu->angle_inited = 1;
        return;
    }

    /*---------- 2) 陀螺仪积分 ----------*/
    gyro_pitch = mpu->pitch + mpu->gx * dt;
    gyro_roll  = mpu->roll  + mpu->gy * dt;
    gyro_yaw   = mpu->yaw   + mpu->gz * dt;

    /*---------- 3) 互补滤波融合 ----------*/
    mpu->pitch = alpha * gyro_pitch + (1.0f - alpha) * acc_pitch;
    mpu->roll  = alpha * gyro_roll  + (1.0f - alpha) * acc_roll;
    mpu->yaw   = gyro_yaw;

    /*---------- 4) 将Yaw限制在 -180~180 或 0~360 均可 ----------
     * 这里限制到 -180~180，便于观察
     */
    if (mpu->yaw > 180.0f)
        mpu->yaw -= 360.0f;
    else if (mpu->yaw < -180.0f)
        mpu->yaw += 360.0f;
}

/*----------------------------------------------------------
 * 获取欧拉角
 *---------------------------------------------------------*/
float MPU6050_Get_Pitch(MPU6050_Attitude_t *mpu)
{
    return mpu->pitch;
}

float MPU6050_Get_Roll(MPU6050_Attitude_t *mpu)
{
    return mpu->roll;
}

float MPU6050_Get_Yaw(MPU6050_Attitude_t *mpu)
{
    return mpu->yaw;
}