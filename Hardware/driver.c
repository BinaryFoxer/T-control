#include "stm32f10x.h"                  // Device header
#include "PWM.h"

// =============================
// 函数名称：驱动电流控制函数
// 函数功能：控制驱动电流的大小方向
// 入口参数：@PWM
// 函数返回：
// 当前版本：VER1.0
// 修改日期：2025
// 当前作者：
// 其它备注：
// ==============================





void driver_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	PWM_Init();
}

void driver_SetPWM(int8_t PWM)
{
	if (PWM >= 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		PWM_SetCompare1(PWM);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
		PWM_SetCompare1(-PWM);
	}
}
