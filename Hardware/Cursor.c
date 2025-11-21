#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Key.h"

uint8_t current_line = 1;
uint8_t Key = 0;
uint16_t Flag = 0;
struct PID
	{	float kp;
		float ki;
		float kd;
	}pid = {0.1,0.0,0.1};

void Cursor_Init(void)
{
	
		OLED_ShowString(1, current_line, ">");
		OLED_ShowFloat(1, 12, pid.kp, 1, 1);
		OLED_ShowFloat(2, 12, pid.ki, 1, 1);
		OLED_ShowFloat(3, 12, pid.kd, 1, 1);
		
}		
	
void Cursor_Tick(void)
{
		Key = Key_GetNum();
		if(Key == 3)
		{
			OLED_ShowString(current_line, 1, " ");
			current_line = ((current_line - 1 + 1) % 4) + 1;
			OLED_ShowString(current_line, 1, ">");
		}
		if(Key == 1)
		{
			switch(current_line){
				case 1: pid.kp += 0.1; break;
				case 2: pid.ki += 0.005; break;
				case 3: pid.kd += 0.1; break;
			}
			OLED_ShowFloat(current_line, 12, 
                      (current_line == 1) ? pid.kp : 
                      (current_line == 2) ? pid.ki : pid.kd, 
                      (current_line == 2) ? 1 : 1, 
                      (current_line == 2) ? 3 : 1);
		}
		if(Key == 2)
		{
			switch(current_line){
				case 1: pid.kp -= 0.1; break;
				case 2: pid.ki -= 0.005; break;
				case 3: pid.kd -= 0.1; break;
			}
			OLED_ShowFloat(current_line, 12, 
                      (current_line == 1) ? pid.kp : 
                      (current_line == 2) ? pid.ki : pid.kd, 
                      (current_line == 2) ? 1 : 1, 
                      (current_line == 2) ? 3 : 1);
		}
		if(Key == 4)
		{
			Flag = 1;
		}
}





