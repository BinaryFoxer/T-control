#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "driver.h"
#include "Serial.h"
#include "RP.h"
#include "Temper.h"
#include "kalman_new.h"

// 1. ADC以较高频率采样（比如每1ms一次），通过DMA存储到缓冲区。

// 2. 每次获得新的ADC采样值时，立即调用卡尔曼滤波更新函数，得到实时的滤波值。

// 3. PID每40ms触发时，读取当前最新的卡尔曼滤波输出作为当前温度值。

int key_num;
int PWM;
int RP_1, RP_2, RP_3, RP_4;
// 目标值，传感器实测值，驱动输出值
float Actual, Target, Out;
float Kp, Ki, Kd;
// 当前误差，前一次误差, 误差积累
float Error0, Error1, ErrorInt;
// Kalman_InitDef kalman;

int main(void)
{
	
	OLED_Init();
	LED_Init();
	Key_Init();
	RP_Init();
	driver_Init();
	Serial_Init();
	GetTemper_Init();
	// Kalman_Parameter_Init(&kalman);
	
	Timer_Init();

	OLED_Printf(0, 0, OLED_6X8, "Temperature Control");
	OLED_Update();

	while (1)
	{
		// Kp = RP_GetValue(1) / 4095.0 * 2;
		// Ki = RP_GetValue(2) / 4095.0 * 2;
		// Kd = RP_GetValue(3) / 4095.0 * 2;
		Kp = 15;
		Ki = 0.0001;
		Kd = 0;
		// 目标温度，先映射到0~50，这个范围可改
		// Target = RP_GetValue(4) / 4095.0 * 50;
		Target = 35;

		OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);
		OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);
		OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);

		OLED_Printf(64, 16, OLED_8X16, "Tar:%04.0f", Target);
		OLED_Printf(64, 32, OLED_8X16, "Act:%04.0f", Actual);
		OLED_Printf(64, 48, OLED_8X16, "Out:%04.0f", Out);

		OLED_Update();

		Serial_Printf("%f, %f, %f\r\n", Target, Actual, Out);

	}

}


void TIM1_UP_IRQHandler(void)
{
	static uint16_t cnt = 0;
	

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick();

		cnt++;
		// pid更新频率暂时设置为40ms，温度值采样间隔也为40ms
		if (cnt >= 40)
		{
			cnt = 0;

			// 这里是温度ADC采集，使用ADC1进行输入即可，打开DMA
			// 采集后的数据进行kalman滤波后再传到此处进行pid计算
			// 这里应该是调用一次函数才进行卡尔曼迭代，但是应该也可以
			// Actual = update_Kalman(&kalman, AD_Value) / 4095.0 * 50;
			Actual = -0.0274 * AD_Value + 81.47;

			// 更新误差值
			Error1 = Error0;
			Error0 = Target - Actual;

			if(Ki != 0)
			{
				ErrorInt += Error0;
			}
			else
			{
				ErrorInt = 0;
			}

			Out = Kp * Error0 + Ki * ErrorInt + Kd * (Error0 - Error1);
			
			// 限幅pid输出值
			if(Out > 30){Out = 100;}
			if(Out < -30){Out = -100;}
			
			// 更新驱动输出
			driver_SetPWM(Out);
		}

		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}

}
