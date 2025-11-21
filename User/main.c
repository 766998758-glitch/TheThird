#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "LED.h"
#include "Timer.h"
#include "Cursor.h"

uint8_t KeyNum;

int main(void)
{
	OLED_Init();
	Key_Init();
	Timer_Init();
	Cursor_Init(); 
	
	OLED_ShowString(1, 3, "KP: ");
	OLED_ShowString(2, 3, "KI: ");
	OLED_ShowString(3, 3, "KD: ");
	OLED_ShowString(4, 3, "Set_Off: ");
	
	while (1)
	{
		
	}
}

void TIM2_IRQHandler(void)     
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Cursor_Tick();
		Key_Tick();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
