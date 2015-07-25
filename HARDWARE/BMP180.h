#ifndef __BMP180_H
#define __BMP180_H
#include "stm32f4xx.h"

#define BMP180_SLAVE_ADDRESS    0xEE		/* I2C从机地址 */

typedef struct
{
	/* 用于保存芯片内部EEPROM的校准参数 */
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
	
	uint8_t OSS;	/* 过采样值，可有用户自己设定 */

	/* 下面2个单元用于存放计算的真实值 */
	int32_t Temp;	/* 温度值， 单位 0.1摄氏度 */
	int32_t Press;	/* 压力值， 单位 Pa */
}BMP180_T;

extern BMP180_T g_tBMP180;

void BMP180_Init(void);
void BMP180_ReadTempPress(void);


void BMP180_InitI2C(void);
void BMP180_i2c_Start(void);
void BMP180_i2c_Stop(void);
void BMP180_i2c_SendByte(uint8_t _ucByte);
uint8_t BMP180_i2c_ReadByte(void);
uint8_t BMP180_i2c_WaitAck(void);
void BMP180_i2c_Ack(void);
void BMP180_i2c_NAck(void);


#define BMP180_I2C_WR	0		/* 写控制bit */
#define BMP180_I2C_RD	1		/* 读控制bit */

#endif
