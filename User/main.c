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
#include <math.h>

// 1. ADC以较高频率采样（比如每1ms一次），通过DMA存储到缓冲区。

// 2. 每次获得新的ADC采样值时，立即调用卡尔曼滤波更新函数，得到实时的滤波值。

// 3. PID每40ms触发时，读取当前最新的卡尔曼滤波输出作为当前温度值。

int key_num;
int PWM;
int RP_1, RP_2, RP_3, RP_4;
// 目标值，传感器实测值，驱动输出值
float Actual, Target, Out, Actual_raw,show;
float Kp, Ki, Kd;
// 当前误差，前一次误差, 误差积累
float Error0, Error1, ErrorInt, final_Int;
Kalman_InitDef kalman;
float dout, C;
int flag = 0;

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
		// if(Key_GetNum() == 1)
		// {
		// 	Kp = 16;
		// 	Ki = 0.0009;
		// 	Kd = 8;
		// }else{
		// 	Kp = 0;
		// 	Ki = 0;
		// 	Kd = 0;
		// }
		Kp = 15.5;
		Ki = 0;
		Kd = 9;
		Target = 35;

		OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);
		OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);
		OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);

		OLED_Printf(64, 16, OLED_8X16, "T:%04.2f", Target);
		OLED_Printf(64, 32, OLED_8X16, "A:%04.2f", Actual);
		OLED_Printf(64, 48, OLED_8X16, "O:%04.2f", Out);

		OLED_Update();

		Serial_Printf("%f, %f, %f, %f\r\n", Target, Actual, Out, final_Int);

	}

}


void TIM1_UP_IRQHandler(void)
{
	static uint16_t cnt = 0;
	static float Actual_prev = 28; // 记录前一次的 Actual 值

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick();

		cnt++;
		// pid更新频率暂时设置为40ms，温度值采样间隔也为40ms
		if (cnt >= 60)
		{
			cnt = 0;

			// 这里是温度ADC采集，使用ADC1进行输入即可，打开DMA
			// 采集后的数据进行kalman滤波后再传到此处进行pid计算
			// 这里应该是调用一次函数才进行卡尔曼迭代，但是应该也可以
			
			//Actual_raw = -0.0274 * AD_Value + 81.47;
			//Actual = update_Kalman(&kalman, Actual_raw);
			Actual = -0.0274 * AD_Value + 81.47;

			// 简单滤波：0.4 * Actual_prev + 0.6 * Actual
			float Filtered_Actual = 0.4 * Actual_prev + 0.6 * Actual;


			// 使用滤波后的值作为实际作用的值
			Actual = Filtered_Actual;
			// 更新 Actual_prev 为当前的 Actual 值
			Actual_prev = Actual;
			
			// 计算 Actual_show
            if (Actual >= 34 && Actual <= 36)
            {
                show = 35 + (Actual - 35) / 10 + 0.05;
            }
            else
            {
                show = Actual;
            }
			// 更新误差值
			Error1 = Error0;
			Error0 = Target - Actual;
			
			// 容忍区间
			if(fabs(Error0) < 0.15)
			{
				ErrorInt = 0;
				Out = 0;
				flag = 1;
			}
			else
			{
				flag = 0;
				// 积分分离
				if(fabs(Error0) < 1.5)
				{
					Ki = 0.0115;
				}
				
				// 变速积分
				//C = 1 / (0.3 * fabs(Error0) + 1);
				
				if(Ki != 0)
				{
					ErrorInt += Error0;
				}
				else
				{
					ErrorInt = 0;
				}
				if(ErrorInt > 80/Ki){ErrorInt = 80/Ki;}
				if(ErrorInt < -80/Ki){ErrorInt = -80/Ki;}
				
				// 微分滤波
				float a = 0.8;
				dout = (1 - a) * Kd * (Error0 - Error1) + a * dout;
				final_Int = Ki * ErrorInt;
				Out = Kp * Error0 + final_Int + dout;

			}
			
			if(fabs(Error0) < 1 && fabs(Error0) > 0.5)
			{	
				Out += 12;
			}else if(fabs(Error0) < 0.5 && fabs(Error0) > 0.3)
			{
				Out += 7;
			}
			
			if(flag == 1)
			{
				Out = 18;	
			}
			
			// 限幅pid输出值
			if(Out > 60){Out = 100;}
			//if(Out < 4){Out = 5;}
			if(Out < -60){Out = -100;}
			
			// 更新驱动输出
			driver_SetPWM(Out);
		}

		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}

}
