#ifndef __HMC5883L_H
#define __HMC5883L_H


#include "stm32f4xx.h"
#include "sys.h" 

//IO方向设置
#define HMC_SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//PB9输入模式
#define HMC_SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //PB9输出模式
//IO操作函数	 
#define HMC_SCL    PBout(6) //SCL
#define HMC_SDA    PBout(7) //SDA	 
#define HMC_READ_SDA   PBin(7)  //输入SDA 


//#define HMC_SCL_H (GPIOB->BSRR=GPIO_Pin_6)
//#define HMC_SCL_L (GPIOB->BRR=GPIO_Pin_6)

//#define HMC_SDA_H (GPIOB->BSRR=GPIO_Pin_7)
//#define HMC_SDA_L (GPIOB->BRR=GPIO_Pin_7)

//#define HMC_SDA (GPIOB->IDR & GPIO_Pin_7)

#define HMC5883L_OFFSET_X (9)
#define HMC5883L_OFFSET_Y (149)

#define HMC5883L_GAIN_Y 10403

u8 HMC5883_Init(void);
void HMC_MultipleRead(u8* pBuffer);
void HMC_GetValue(s16* pBuffer);

#endif



