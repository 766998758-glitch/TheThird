
#include "Cursor.h"
#include "Key.h"
#include "OLED.h"

uint8_t current_line = 1;
uint8_t Key = 0;
uint16_t Flag = 0;
struct PID pid = {5, 0.234, 1.5}; //先用这个，之后再调整


void Cursor_Init(void)
{
	  /*菜单*/
    OLED_ShowString(current_line, 1, ">");
    OLED_ShowFloat(1, 12, pid.kp, 1, 1);
    OLED_ShowFloat(2, 12, pid.ki, 1, 3);  
    OLED_ShowFloat(3, 12, pid.kd, 1, 1);
    OLED_ShowString(4, 12, "STOP");  
}

void Cursor_Tick(void)
{
    Key = Key_GetNum();
    
    if(Key == 3)  //上下移动
    {
        OLED_ShowString(current_line, 1, " "); 
        current_line = (current_line % 4) + 1;   //取余魔术 1->2->3->4->1
        OLED_ShowString(current_line, 1, ">");  
    }
    
    if(Key == 1)  //pid++
    {
        switch(current_line){
            case 1: 
                pid.kp += 0.1; 
                break;
            case 2: 
                pid.ki += 0.005;   
                break;
            case 3: 
                pid.kd += 0.1; 
                break;
        }
        //区别于第二轮的更新显示方法。很明显这个可移植性更高。
        OLED_ShowFloat(current_line, 12, 
                      (current_line == 1) ? pid.kp : 
                      (current_line == 2) ? pid.ki : pid.kd, 
                      (current_line == 2) ? 1 : 1, 
                      (current_line == 2) ? 3 : 1);
    }
    
    if(Key == 2)  //pid--
    {
        switch(current_line){
            case 1: 
                pid.kp -= 0.1; 
                break;
            case 2: 
                pid.ki -= 0.005; 
                break;
            case 3: 
                pid.kd -= 0.1; 
                break;
        }
        OLED_ShowFloat(current_line, 12, 
                      (current_line == 1) ? pid.kp : 
                      (current_line == 2) ? pid.ki : pid.kd, 
                      (current_line == 2) ? 1 : 1, 
                      (current_line == 2) ? 3 : 1);
    }
    
    if(Key == 4)  
    {
        Flag = 1;  //发车
    }
}
