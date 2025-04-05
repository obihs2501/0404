#include "stm32f10x.h"
#include "exti.h"

/**************************************************************************
�������ܣ��ⲿ�жϳ�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/



void TIM7_EXTI_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // ʹ��TIM6ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    // ��ʱ��TIM6������������
    TIM_TimeBaseStructure.TIM_Period = 10000-1; // �Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = 7200-1; // Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ʱ�ӷָ�
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;			
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    // ʹ��TIM6�����ж�
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//    // NVIC����

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; // TIM6�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);

    // ʹ��TIM6
    TIM_Cmd(TIM7, ENABLE);
}

// TIM6�жϷ�����


void TIM6_EXTI_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   
    // ʹ��TIM6ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    // ��ʱ��������������
    TIM_TimeBaseStructure.TIM_Period = 100 - 1; // �Զ�����ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = 720 - 1; // Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
		TIM_ClearFlag(TIM6, TIM_FLAG_Update);

    // ʹ��TIM6�ж�
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    // ����NVIC
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	  NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // ʹ��TIM6
    TIM_Cmd(TIM6, ENABLE);
}










