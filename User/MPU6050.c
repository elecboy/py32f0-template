#include <stddef.h>

#if defined(GD32E23x)
#include "gd32e23x.h"
#elif defined(GD32L23x)
#include "gd32l23x.h"
#elif defined(PY32F030x8)
#else
#error "Unknown chip series"
#endif

#include "I2C_Libary.h"
#include "MPU6050.h"

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0xEB	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress	0x68	//IIC地址
int16_t Acc[3] = {0}, GYO[3] = {0};
int32_t Gyo_Offset[3] = {0};
void MPU6050_Read()
{
	// i2c_read(NULL, ICN6211_ADDRESS7, 0x0, icn6211_reg_buf, 0xe3);
	i2c_read(NULL,SlaveAddress,ACCEL_XOUT_H,(uint8_t*)Acc,2);
	LL_mDelay(1);
	i2c_read(NULL,SlaveAddress,ACCEL_YOUT_H,(uint8_t*)(Acc + 1),2);
	LL_mDelay(1);
	i2c_read(NULL,SlaveAddress,ACCEL_ZOUT_H,(uint8_t*)(Acc + 2),2);
	LL_mDelay(1);
	i2c_read(NULL,SlaveAddress,GYRO_XOUT_H,(uint8_t*)GYO,2);
	LL_mDelay(1);
	GYO[0] -= Gyo_Offset[0];
	i2c_read(NULL,SlaveAddress,GYRO_YOUT_H,(uint8_t*)(GYO + 1),2);
	LL_mDelay(1);
	GYO[1] -= Gyo_Offset[1];
	i2c_read(NULL,SlaveAddress,GYRO_ZOUT_H,(uint8_t*)(GYO + 2),2);
	LL_mDelay(1);
	GYO[2] -= Gyo_Offset[2];
}
void MPU6050_GYO_Calibration()
{
	uint8_t i , k;
	for(k = 0;k < 10;k++)
	{
		uint8_t i = 0;
		i2c_read(NULL,SlaveAddress,GYRO_XOUT_H,(uint8_t*)GYO,2);
		i2c_read(NULL,SlaveAddress,GYRO_YOUT_H,(uint8_t*)(GYO + 1),2);
		i2c_read(NULL,SlaveAddress,GYRO_ZOUT_H,(uint8_t*)(GYO + 2),2);
		for(i = 0;i<3;i++)
		{
			Gyo_Offset[i] += GYO[i];
		}
	}
	for(i = 0;i<3;i++)
	{
		Gyo_Offset[i] /= 10;
	}
}
void MPU6050_Config()
{
	uint8_t Value = 0;
	Value = 0x00;
	i2c_write(NULL,SlaveAddress,PWR_MGMT_1,&Value,1);
	Value = 0x07;
	i2c_write(NULL,SlaveAddress,SMPLRT_DIV,&Value,1);
	Value = 0x04;
	i2c_write(NULL,SlaveAddress,CONFIG,&Value,1);
	Value = 0x10;//1000 degree
	i2c_write(NULL,SlaveAddress,GYRO_CONFIG,&Value,1);
	Value = 0x09;//4g
	i2c_write(NULL,SlaveAddress,ACCEL_CONFIG,&Value,1);
	MPU6050_GYO_Calibration();
}
