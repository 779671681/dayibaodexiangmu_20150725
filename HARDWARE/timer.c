#include "timer.h"
#include "led.h"
#include "adc.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

#define true 1
#define false 0
int BPM;                   // used to hold the pulse rate
int Signal;                // holds the incoming raw data
int IBI = 600;             // holds the time between beats, must be seeded! 
unsigned char Pulse = false;     // true when pulse wave is high, false when it's low
unsigned char QS = false;        // becomes true when Arduoino finds a beat.
int rate[10];                    // array to hold last ten IBI values
unsigned long sampleCounter = 0;          // used to determine pulse timing
unsigned long lastBeatTime = 0;           // used to find IBI
int P =3000;                      // used to find peak in pulse wave, seeded
int T = 3000;                     // used to find trough in pulse wave, seeded
int thresh = 3000;                // used to find instant moment of heart beat, seeded
int thresh_high = 3700;          //3.2V
int thresh_low = 3000;           //2.4V
int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
int Num;
unsigned char firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
unsigned char secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	unsigned int runningTotal;
	int i;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
				Signal=Get_Adc_HeartRate();				 // read the Pulse Senso
				sampleCounter += 2;                         // keep track of the time in mS with this variable
				Num = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise
//				HAL_ADC_Start(&hadc1);									//restart ADC conversion

				//  find the peak and trough of the pulse wave
			if(Signal < thresh_low && Num > (IBI*3)/5){       // avoid dichrotic noise by waiting 3/5 of last IBI
				if (Signal < T){                        // T is the trough
					T = Signal;                         // keep track of lowest point in pulse wave 
				}
			}

			if(Signal > thresh_high && Signal > P){          // thresh condition helps avoid noise
				P = Signal;                             // P is the peak
			}                                        // keep track of highest point in pulse wave

			//  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
			// signal surges up in value every time there is a pulse
			if (Num > 250){                                   // avoid high frequency noise
				if ( (Signal > thresh_high) && (Pulse == false) && (Num > (IBI*3)/5) ){        
					Pulse = true;                               // set the Pulse flag when we think there is a pulse
	//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);                // turn on pin 13 LED
					IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
					lastBeatTime = sampleCounter;               // keep track of time for next pulse

					if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
						secondBeat = false;                  // clear secondBeat flag
						for(i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
							rate[i] = IBI;                      
						}
					}

					if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
						firstBeat = false;                   // clear firstBeat flag
						secondBeat = true;                   // set the second beat flag
		 //       sei();                               // enable interrupts again
						return;                              // IBI value is unreliable so discard it
					}   


					// keep a running total of the last 10 IBI values
					runningTotal = 0;                  // clear the runningTotal variable    

					for(i=0; i<=8; i++){                // shift data in the rate array
						rate[i] = rate[i+1];                  // and drop the oldest IBI value 
						runningTotal += rate[i];              // add up the 9 oldest IBI values
					}

					rate[9] = IBI;                          // add the latest IBI to the rate array
					runningTotal += rate[9];                // add the latest IBI to runningTotal
					runningTotal /= 10;                     // average the last 10 IBI values 
					BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
					QS = true;                              // set Quantified Self flag 
					// QS FLAG IS NOT CLEARED INSIDE THIS ISR
				}                       
			}

			if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
//				 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);            // turn off pin 13 LED
				Pulse = false;                         // reset the Pulse flag so we can do it again
				amp = P - T;                           // get amplitude of the pulse wave
				thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
				P = thresh_high;                            // reset these for next time
				T = thresh_low;
			}

			if (Num > 2500){                           // if 2.5 seconds go by without a beat
				thresh = (512*4);                          // set thresh default
				P = thresh_high;                               // set P default
				T = thresh_low;                               // set T default
				lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date   
				BPM = 0;
				firstBeat = true;                      // set these to avoid noise
				secondBeat = false;                    // when we get the heartbeat back
				
			}

			
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
