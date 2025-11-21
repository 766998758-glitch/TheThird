#include "stm32f10x.h"                  // Device header
#include "Delay.h"



uint8_t Key_Num;

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructureB;
	GPIO_InitStructureB.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructureB.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructureB.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructureB);
	
	GPIO_InitTypeDef GPIO_InitStructureA;
	GPIO_InitStructureA.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructureA.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStructureA.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructureA);
}

uint8_t Key_GetNum(void)    
{
	uint8_t temp;  
	temp = Key_Num;
	Key_Num = 0;
	return temp;
}



uint8_t Key_GetState(void) //获取当前按键状态
{
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 0)  //说明K1被按下
	{
		return 1;
	}
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 0)  //说明K2被按下
	{
		return 2;
	}
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0)  //说明K1被按下
	{
		return 3;
	}
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0)  //说明K2被按下
	{
		return 4;
	}
		return 0;   //如果没有任何一个按键按下
}


void Key_Tick(void){        
	static uint8_t count;
	static uint8_t CurrState, PrevState;   
	count++;
	
	if(count>=20)      
	{
		count=0;
		
		PrevState = CurrState;
		CurrState = Key_GetState();
		
		if(CurrState == 0 && PrevState != 0)  //捕捉到一次按键按下再松手
		{
			Key_Num = PrevState;     //K1/2按下的松手瞬间，Key_Num的值编变成1/2
		}
	}
	
}






















