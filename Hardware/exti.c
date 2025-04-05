#include "stm32f10x.h"
#include "exti.h"

/**************************************************************************
函数功能：外部中断初始化
入口参数：无
返回  值：无 
**************************************************************************/



void TIM7_EXTI_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能TIM6时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    // 定时器TIM6基本参数配置
    TIM_TimeBaseStructure.TIM_Period = 10000-1; // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 7200-1; // 预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;			
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    // 使能TIM6更新中断
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//    // NVIC配置

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; // TIM6中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);

    // 使能TIM6
    TIM_Cmd(TIM7, ENABLE);
}

// TIM6中断服务函数


void TIM6_EXTI_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   
    // 使能TIM6时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    // 定时器基本参数配置
    TIM_TimeBaseStructure.TIM_Period = 100 - 1; // 自动重载值
    TIM_TimeBaseStructure.TIM_Prescaler = 720 - 1; // 预分频器
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
		TIM_ClearFlag(TIM6, TIM_FLAG_Update);

    // 使能TIM6中断
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    // 配置NVIC
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	  NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 使能TIM6
    TIM_Cmd(TIM6, ENABLE);
}










