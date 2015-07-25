#include "HMC5883L.h"
#include "stm32f4xx.h"
#include "delay.h"
#define MMA_ADR 0x3c
u32 I2CStep;
void HMC_Delay(void)
{
//	I2CStep=10;
//	while(I2CStep--);
	delay_us(4);
}
void HMC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE);


	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

}
u8 HMC_I2C_Start(void)
{
	HMC_SDA_OUT();     //sda线输出
	HMC_SDA = 1;
	HMC_Delay();
	HMC_SCL = 1;
	HMC_Delay();
	HMC_SDA_IN();
	if(!HMC_READ_SDA) return 0;//SDA线为低电平则总线忙,退出
	HMC_SDA = 0;;
	return 1;
}
void HMC_I2C_Stop(void)
{
	HMC_SDA_OUT();     //sda线输出
	HMC_SCL = 0;;
	HMC_Delay();
	HMC_SDA = 0;;
	HMC_Delay();
	HMC_SCL = 1;;
	HMC_Delay();
	HMC_SDA = 1;;
	HMC_Delay();
}
void HMC_I2C_Ack(void)
{
	HMC_SDA_OUT();   
	HMC_SCL = 0;;
	HMC_Delay();
	HMC_SDA = 0;;
	HMC_Delay();
	HMC_SCL = 1;;
	HMC_Delay();
	HMC_SCL = 0;;
	HMC_Delay();
}
void HMC_I2C_NAck(void)
{
	HMC_SDA_OUT();  
	HMC_SCL = 0;;
    HMC_Delay();
    HMC_SDA = 1;;
    HMC_Delay();
    HMC_SCL = 1;;
    HMC_Delay();
    HMC_SCL = 0;;
    HMC_Delay();
}
u8 HMC_I2C_WaitAck(void)
{
	HMC_SDA_IN();      //SDA设置为输入 
	HMC_SCL = 0;;
    HMC_Delay();
    HMC_SDA = 1;;			
    HMC_Delay();
    HMC_SCL = 1;;
    HMC_Delay();
    if(HMC_READ_SDA)
    {
			HMC_SCL = 0;
			HMC_SDA_OUT();      
			return 0;
    }
    HMC_SCL = 0;;
    return 1;
}
void HMC_I2C_SendByte(u8 SendByte) //数据从高位到低位
{
	u8 i=8;
	HMC_SDA_OUT();      
    
  while(i--)
	{
		HMC_SCL = 0;;
		HMC_Delay();
		if(SendByte&0x80)
			HMC_SDA = 1;
		else
			HMC_SDA = 0;
		SendByte<<=1;
		HMC_Delay();
		HMC_SCL = 1;
		HMC_Delay();
	}
	HMC_SCL = 0;;
}
u8 HMC_I2C_ReceiveByte(void)  //数据从高位到低位//
{ 
	u8 i=8;
	u8 ReceiveByte=0;
	HMC_SDA_IN();    

	HMC_SDA = 1;			
	while(i--)
	{
		ReceiveByte<<=1;      
		HMC_SCL = 0;;
		HMC_Delay();
		HMC_SCL = 1;;
		HMC_Delay();	
		if(HMC_READ_SDA)
		{
			ReceiveByte|=0x01;
		}
	}
	HMC_SCL = 0;;
	return ReceiveByte;
}
u8 HMC_I2C_Write(u8 address,u8 data)
{
	HMC_I2C_Start();
	HMC_I2C_SendByte(MMA_ADR);//设置器件地址
	HMC_I2C_Ack();
	HMC_I2C_SendByte(address);   //设置低起始地址      
	HMC_I2C_Ack();	
	HMC_I2C_SendByte(data);
	HMC_I2C_Ack();   
	HMC_I2C_Stop(); 
	//注意：因为这里要等待EEPROM写完，可以采用查询或延时方式(10ms)
	delay_ms(10);
	return 1;	
}

u8 HMC_I2C_Read(u8 address)//读字节
{
	u8 temp=0;
	HMC_I2C_Start();
	HMC_I2C_SendByte(MMA_ADR);//设置器件地址
	HMC_I2C_Ack();
	HMC_I2C_SendByte(address);   //设置低起始地址      
	HMC_I2C_Ack();
	HMC_I2C_Start();
	HMC_I2C_SendByte(MMA_ADR|0x01);//设置器件地址
	HMC_I2C_Ack();
	temp=HMC_I2C_ReceiveByte();
	HMC_I2C_Ack();
	HMC_I2C_Stop();
	return temp;
}

void HMC_MultipleRead(u8* pBuffer)
{
	u8 i=0;
	HMC_I2C_Start();
	HMC_I2C_SendByte(0x3c);
	HMC_I2C_WaitAck();
	HMC_I2C_SendByte(0x03);
	HMC_I2C_WaitAck();
	HMC_I2C_Start();
	HMC_I2C_SendByte(0x3c+1);
	HMC_I2C_WaitAck();
	for(i=0;i<6;i++)
	{
		pBuffer[i]=HMC_I2C_ReceiveByte();
		if(i == 5)
			HMC_I2C_NAck();
		else
			HMC_I2C_Ack();
	}
	
	HMC_I2C_Stop();
}
void HMC_GetValue(s16* pBuffer)
{
	u8 tmp[6];
//	s32 s32Val;
	HMC_MultipleRead(tmp);
	
//	pBuffer[0]=(s16)(((s16)tmp[0] << 8) | tmp[1])+HMC5883L_OFFSET_X;
//	s32Val = (s16)(((s16)tmp[4] << 8) | tmp[5])+HMC5883L_OFFSET_Y;
//	s32Val = (s32Val*HMC5883L_GAIN_Y)/10000;
//	pBuffer[1] = (s16)s32Val;
//	pBuffer[2] = (s16)(((s16)tmp[2] << 8) | tmp[3]);
	pBuffer[0]=(s16) ( (((u16)tmp[0])<<8) + tmp[1] );
	pBuffer[1]=(s16) ( (((u16)tmp[2])<<8) + tmp[3] );
	pBuffer[2]=(s16) ( (((u16)tmp[4])<<8) + tmp[5] );
}


u8 HMC5883_Init(void)
{
	HMC_GPIO_Config();
	HMC_I2C_Write(0x02,0x00);//开始转换
//	HMC_I2C_Write(0x01,0xe0);
	return HMC_I2C_Read(10);
}




