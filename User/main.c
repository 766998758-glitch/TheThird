#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "LED.h"
#include "Timer.h"
#include "Cursor.h"
#include "Motor.h"
#include "ADC.h"
#include "PID.h"

//全局变量
PID_Controller line_pid;
uint8_t line_following_enabled = 0;

int main(void)
{
    //系统初始化
    OLED_Init();
    Key_Init();
    Timer_Init();
    Motor_Init();
    ADC1_Module_Init();
		Cursor_Init();
    
    //显示静态界面
    OLED_ShowString(1, 3, "KP: ");
    OLED_ShowString(2, 3, "KI: ");
    OLED_ShowString(3, 3, "KD: ");
    OLED_ShowString(4, 3, "Set_Off: ");
    
    
    //PID控制器初始化（使用Cursor.c里的初始值）
    PID_Init(&line_pid, pid.kp, pid.ki, pid.kd, 1000);
    
    
    while(1)
    {
        //检查发车标志
        if(Flag == 1) {
            line_following_enabled = 1;
            Flag = 0;  //记得清零
            OLED_ShowString(4, 12, "RUN ");  //运行状态
        }
        
        // 循迹控制
        if(line_following_enabled) {
            //使用Cursor.c里的数据实时控制
            line_pid.Kp = pid.kp;
            line_pid.Ki = pid.ki;
            line_pid.Kd = pid.kd;
            
            LineFollower_Update(&line_pid, 600);  //给个基础速度600
        } else {
            Car_Stop();
        }
        
        Delay_ms(10);  //稍微延时让系统稳定一下
    }
}

void TIM2_IRQHandler(void)     
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        Cursor_Tick();  //处理光标移动&PID参数传出
        Key_Tick();     //按键扫描
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
