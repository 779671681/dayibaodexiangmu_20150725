#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "adc.h"
#include "dht11.h"
#include "HMC5883L.h"
#include "BMP180.h"
#include "math.h"
#include "timer.h"

s16 HMC_Buff[3];
extern BMP180_T g_tBMP180;;
extern int BPM; //����
extern int IBI;
extern int P;                      // used to find peak in pulse wave, seeded
extern int T;                     // used to find trough in pulse wave, seeded
extern int thresh;                // used to find instant moment of heart beat, seeded
extern int amp;                   // used to hold amplitude of pulse waveform, seeded
int main(void)
{ 
	u8 t=0;			    
	u8 temperature;  	    
	u8 humidity;
  float angle;  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	USART2_Init(115200);
	  Adc_Init();	        //ADC��ʼ��
	TIM3_Int_Init(20-1,8400-1);	//��ʱ��ʱ��84M����Ƶϵ��16������84M/84=10Khz�ļ���Ƶ�ʣ�����20��Ϊ2ms     
	LED_Init();					//��ʼ��LED 
 	LCD_Init();					//LCD��ʼ��   

	
//	HMC5883_Init();
//	BMP180_Init();
	
// 	while(DHT11_Init())	//DHT11��ʼ��	
//	{
//		LCD_ShowString(30,130,200,16,16,"DHT11 Error");
//		delay_ms(200);
//		LCD_Fill(30,130,239,130+16,WHITE);
// 		delay_ms(200);
//	}								   


printf("hello:\n");
	while(1)
	{	    	    
 		if(t%100==0)//ÿ100ms��ȡһ��
		{							
		//HMC	
//				HMC_GetValue(HMC_Buff);
//				printf("HMC_Value:");
//			  printf("X:%d  ",HMC_Buff[0]);
//			  printf("Y:%d  ",HMC_Buff[2]);
//			  printf("Z:%d  ",HMC_Buff[1]);
//			 angle= atan2((double)HMC_Buff[2],(double)HMC_Buff[0]) * (180 / 3.14159265) + 180; // angle in degrees
//				printf("angle:%f  ",angle);
//			  printf("\n");
//			
//			BMP180_ReadTempPress();
//			printf("BMP180: ");
//			printf("Press:%d  ",(g_tBMP180.Press));
//			printf("Temp:%f  ",(float)(g_tBMP180.Temp)/100);
//			printf("\n");
//			
//			
//			printf("DHT11: ");
//			DHT11_Read_Data(&temperature,&humidity);		//��ȡ��ʪ��ֵ	
//      printf("temperature:");			
//			LCD_ShowNum(30+40,150,temperature,2,16);		//��ʾ�¶�	   		
//			printf("humidity:");						
//			LCD_ShowNum(30+40,170,humidity,2,16);			//��ʾʪ��	 
//      printf("\n");		

printf("BMP: %d IBI:%d P:%d T:%d thresh:%d amp:%d",BPM,IBI,P,T,thresh,amp);
			printf("\n");		
			
			
		}				   
	 	delay_ms(10);
		t++;
		if(t>100)t=0;
//		if(t==20)
//		{
//			t=0;
//			LED0=!LED0;
//		}
	}
}






