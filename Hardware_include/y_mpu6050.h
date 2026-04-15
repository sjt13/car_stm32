#ifndef _Y_MPU6050_H
#define _Y_MPU6050_H

#include "y_myiic.h"
#include "main.h"
#include <math.h>
#include <string.h>

/*----------------------------------------------------------
 * MPU6050 寄存器地址定义
 *---------------------------------------------------------*/
//#define MPU_ACCEL_OFFS_REG      0X06
//#define MPU_PROD_ID_REG         0X0C
#define MPU_SELF_TESTX_REG      0X0D
#define MPU_SELF_TESTY_REG      0X0E
#define MPU_SELF_TESTZ_REG      0X0F
#define MPU_SELF_TESTA_REG      0X10
#define MPU_SAMPLE_RATE_REG     0X19
#define MPU_CFG_REG             0X1A
#define MPU_GYRO_CFG_REG        0X1B
#define MPU_ACCEL_CFG_REG       0X1C
#define MPU_MOTION_DET_REG      0X1F
#define MPU_FIFO_EN_REG         0X23
#define MPU_I2CMST_CTRL_REG     0X24
#define MPU_I2CSLV0_ADDR_REG    0X25
#define MPU_I2CSLV0_REG         0X26
#define MPU_I2CSLV0_CTRL_REG    0X27
#define MPU_I2CSLV1_ADDR_REG    0X28
#define MPU_I2CSLV1_REG         0X29
#define MPU_I2CSLV1_CTRL_REG    0X2A
#define MPU_I2CSLV2_ADDR_REG    0X2B
#define MPU_I2CSLV2_REG         0X2C
#define MPU_I2CSLV2_CTRL_REG    0X2D
#define MPU_I2CSLV3_ADDR_REG    0X2E
#define MPU_I2CSLV3_REG         0X2F
#define MPU_I2CSLV3_CTRL_REG    0X30
#define MPU_I2CSLV4_ADDR_REG    0X31
#define MPU_I2CSLV4_REG         0X32
#define MPU_I2CSLV4_DO_REG      0X33
#define MPU_I2CSLV4_CTRL_REG    0X34
#define MPU_I2CSLV4_DI_REG      0X35

#define MPU_I2CMST_STA_REG      0X36
#define MPU_INTBP_CFG_REG       0X37
#define MPU_INT_EN_REG          0X38
#define MPU_INT_STA_REG         0X3A

#define MPU_ACCEL_XOUTH_REG     0X3B
#define MPU_ACCEL_XOUTL_REG     0X3C
#define MPU_ACCEL_YOUTH_REG     0X3D
#define MPU_ACCEL_YOUTL_REG     0X3E
#define MPU_ACCEL_ZOUTH_REG     0X3F
#define MPU_ACCEL_ZOUTL_REG     0X40

#define MPU_TEMP_OUTH_REG       0X41
#define MPU_TEMP_OUTL_REG       0X42

#define MPU_GYRO_XOUTH_REG      0X43
#define MPU_GYRO_XOUTL_REG      0X44
#define MPU_GYRO_YOUTH_REG      0X45
#define MPU_GYRO_YOUTL_REG      0X46
#define MPU_GYRO_ZOUTH_REG      0X47
#define MPU_GYRO_ZOUTL_REG      0X48

#define MPU_I2CSLV0_DO_REG      0X63
#define MPU_I2CSLV1_DO_REG      0X64
#define MPU_I2CSLV2_DO_REG      0X65
#define MPU_I2CSLV3_DO_REG      0X66

#define MPU_I2CMST_DELAY_REG    0X67
#define MPU_SIGPATH_RST_REG     0X68
#define MPU_MDETECT_CTRL_REG    0X69
#define MPU_USER_CTRL_REG       0X6A
#define MPU_PWR_MGMT1_REG       0X6B
#define MPU_PWR_MGMT2_REG       0X6C
#define MPU_FIFO_CNTH_REG       0X72
#define MPU_FIFO_CNTL_REG       0X73
#define MPU_FIFO_RW_REG         0X74
#define MPU_DEVICE_ID_REG       0X75

/*----------------------------------------------------------
 * MPU6050 IIC 地址
 * AD0接GND -> 0x68
 * AD0接3.3V -> 0x69
 *---------------------------------------------------------*/
#define MPU_ADDR                0X68

/*----------------------------------------------------------
 * 常用量程下的灵敏度
 * 当前初始化采用：
 *  陀螺仪：±2000 dps   -> 16.4 LSB/(°/s)
 *  加速度：±2g        -> 16384 LSB/g
 *---------------------------------------------------------*/
#define MPU_GYRO_SENS_2000DPS   16.4f
#define MPU_ACCEL_SENS_2G       16384.0f

/* 弧度角度转换 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG              (57.2957795f)
#define DEG_TO_RAD              (0.0174532925f)

/*----------------------------------------------------------
 * 互补滤波参数
 * alpha 越大，越依赖陀螺仪；越小，越依赖加速度计
 * 常用 0.95 ~ 0.99
 *---------------------------------------------------------*/
#define MPU_COMPL_FILTER_ALPHA  0.98f

/*----------------------------------------------------------
 * 姿态数据结构体
 *---------------------------------------------------------*/
typedef struct
{
    /* 原始数据 */
    int16_t acc_raw[3];     // X Y Z 加速度原始值
    int16_t gyro_raw[3];    // X Y Z 陀螺仪原始值

    /* 物理量 */
    float ax;               // 单位：g
    float ay;               // 单位：g
    float az;               // 单位：g

    float gx;               // 单位：°/s
    float gy;               // 单位：°/s
    float gz;               // 单位：°/s

    /* 零偏 */
    float gyro_offset_x;    // 陀螺仪X轴零偏，单位：°/s
    float gyro_offset_y;    // 陀螺仪Y轴零偏，单位：°/s
    float gyro_offset_z;    // 陀螺仪Z轴零偏，单位：°/s

    /* 欧拉角 */
    float pitch;            // 俯仰角，单位：°
    float roll;             // 横滚角，单位：°
    float yaw;              // 偏航角，单位：°

    /* 上电初始完成标志 */
    uint8_t angle_inited;
} MPU6050_Attitude_t;

/* 全局姿态对象 */
extern MPU6050_Attitude_t g_mpu6050;

/*----------------------------------------------------------
 * 基础驱动接口
 *---------------------------------------------------------*/
u8 MPU_Init(void);
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf);
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf);
u8 MPU_Write_Byte(u8 reg, u8 data);
u8 MPU_Read_Byte(u8 reg);

u8 MPU_Set_Gyro_Fsr(u8 fsr);
u8 MPU_Set_Accel_Fsr(u8 fsr);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU_Set_Rate(u16 rate);
u8 MPU_Set_Fifo(u8 sens);

short MPU_Get_Temperature(void);
void MPU_Get_Gyroscope(int16_t *pbuf);
void MPU_Get_Accelerometer(int16_t *pbuf);

/*----------------------------------------------------------
 * 新增：姿态解算相关接口
 *---------------------------------------------------------*/

/* 读取原始值并换算成物理量 */
void MPU6050_Read_All(MPU6050_Attitude_t *mpu);

/* 陀螺仪零偏校准
 * times: 采样次数，建议 500 ~ 2000
 * 使用前请保证模块静止
 */
void MPU6050_Gyro_Calibration(MPU6050_Attitude_t *mpu, uint16_t times);

/* 初始化姿态角（通常在校准后调用一次） */
void MPU6050_Angle_Init(MPU6050_Attitude_t *mpu);

/* 姿态更新函数
 * dt：两次调用间隔，单位秒，例如 0.01f 表示 10ms 调一次
 */
void MPU6050_Update_Angle(MPU6050_Attitude_t *mpu, float dt);

/* 获取欧拉角 */
float MPU6050_Get_Pitch(MPU6050_Attitude_t *mpu);
float MPU6050_Get_Roll(MPU6050_Attitude_t *mpu);
float MPU6050_Get_Yaw(MPU6050_Attitude_t *mpu);

#endif
