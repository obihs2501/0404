#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Servo.h" 
#include <math.h>
#include <stdlib.h>
// ����״̬
#define STATE_ACTION_1 1
#define STATE_ACTION_2 2
#define STATE_ACTION_3 3
#define STATE_ACTION_4 4
#define STATE_ACTION_5 5
#define STATE_ACTION_6 6
#define STATE_ACTION_7 7
#define STATE_ACTION_8 8
#define STATE_ACTION_9 9
#define STATE_ACTION_10 10

//������̬
//	SERVO_SetAngle(1,1,99);
//	SERVO_SetAngle(1,2,86.4);	
//	SERVO_SetAngle(1,3,29.7);	
//	SERVO_SetAngle(1,4,301.5);

//������̬
//	SERVO_SetAngle(1,1,84);
//	SERVO_SetAngle(1,2,90);	
//	SERVO_SetAngle(1,3,290);	
//	SERVO_SetAngle(1,4,36);		
//	



uint8_t action_executed = 1;

void Climbing_Action_Perform1(void) {
	
 static uint8_t current_state = STATE_ACTION_1;

    switch (current_state) {
        case STATE_ACTION_1:
            // ִ�е�1������
						SERVO_SetAngle(1, 4, 36);
						SERVO_SetAngle(1, 2, 90);
				    SERVO_SetAngle(1, 3, 290);
						SERVO_SetAngle(1, 1, 84);
            Delay_ms(2000);
            current_state = STATE_ACTION_2;
            // ע������û�� break������ִ����һ�� case
        case STATE_ACTION_2:
            // ִ�е�2������
            SERVO_SetAngle(1, 4, 54);
				    Delay_ms(1000);
						SERVO_SetAngle(1, 2, 48);
				    SERVO_SetAngle(1, 3, 290.7);
						SERVO_SetAngle(1, 1, 90);
            Delay_ms(2000);
            current_state = STATE_ACTION_3;
        case STATE_ACTION_3:
            // ִ�е�3������
            SERVO_SetAngle(1, 4, 108);
				    Delay_ms(1500);
						SERVO_SetAngle(1, 2, 167);
            SERVO_SetAngle(1, 3, 290.7);
						SERVO_SetAngle(1, 1, 90);				    
				    current_state = STATE_ACTION_4;
				case STATE_ACTION_4:
            // ִ�е�4������
            SERVO_SetAngle(1, 4, 108);			    
						SERVO_SetAngle(1, 2, 167);
            SERVO_SetAngle(1, 3, 290.7);
						SERVO_SetAngle(1, 1, 90);
            Delay_ms(2000);
            current_state = STATE_ACTION_5;
				case STATE_ACTION_5:
            // ִ�е�5������
            SERVO_SetAngle(1, 4, 108);
						SERVO_SetAngle(1, 2, 167);				 
    				SERVO_SetAngle(1, 3, 254.7);
						SERVO_SetAngle(1, 1, 138);
            Delay_ms(2000);
            current_state = STATE_ACTION_6;
				case STATE_ACTION_6:
            // ִ�е�6������
            SERVO_SetAngle(1, 4, 108);		    
						SERVO_SetAngle(1, 2, 167);
            SERVO_SetAngle(1, 3, 218.7);
				    Delay_ms(2000);
						SERVO_SetAngle(1, 1, 36);		
//            current_state = STATE_ACTION_7;
//				case STATE_ACTION_7:
//            //����7���������ǰ��
//				    Delay_ms(2000);
//				    Delay_ms(2000);
//            current_state = STATE_ACTION_8;
//				case STATE_ACTION_8:
//					// ִ�е�8������
//				    SERVO_SetAngle(1, 4, 108);
//            SERVO_SetAngle(1, 3, 218.7);	
//				    Delay_ms(2000);				
// 						SERVO_SetAngle(1, 1, 40);
//						SERVO_SetAngle(1, 2, 163);
//				    Delay_ms(2000);
//            current_state = STATE_ACTION_9;
//				case STATE_ACTION_9:
//					// ִ�е�9������
// 						SERVO_SetAngle(1, 1, 108);
//						SERVO_SetAngle(1, 2, 75);
// 				    Delay_ms(2000);	   				
//				    SERVO_SetAngle(1, 4, 36);
//            SERVO_SetAngle(1, 3, 290);	
//				    current_state = STATE_ACTION_10;
//					case STATE_ACTION_10:
//					// ִ�е�10������
//				    SERVO_SetAngle(1, 4, 36);
//            SERVO_SetAngle(1, 3, 290);
// 				    Delay_ms(2000);	   				
//					  SERVO_SetAngle(1, 1, 84);
//						SERVO_SetAngle(1, 2, 90);		
						action_executed = 0;//ע���ʼ����1���������һ��������0�������ٴ�ʹ�����׶�����ͨ������־λ��λ
            break;
        default:
            break;
    }
}


 uint8_t transform_flag;  // �����־

void TransformForwardToReverse(void)
{
    // ���÷�����̬
      SERVO_SetAngle(1, 1, 99);
      SERVO_SetAngle(1, 3, 29.7);	   
     	SERVO_SetAngle(1, 2, 90);
      SERVO_SetAngle(1, 4, 36);  
		    Delay_ms(2000);
      SERVO_SetAngle(1, 1, 99);
      SERVO_SetAngle(1, 3, 29.7);	
	    SERVO_SetAngle(1, 2, 86.4);
      SERVO_SetAngle(1, 4, 301.5);   		  

	    transform_flag = 0;  // �����־
}

// ����������̬�任����
void TransformReverseToForward(void)
{
    // ����������̬
      SERVO_SetAngle(1, 1, 84);
      SERVO_SetAngle(1, 3, 290);
	    SERVO_SetAngle(1, 2, 86);
      SERVO_SetAngle(1, 4, 301.5); 	
	  	Delay_ms(2000);
		  Delay_ms(2000);
		  Delay_ms(2000);
			SERVO_SetAngle(1, 2, 90);
			SERVO_SetAngle(1, 4, 36);
}


// ��ʼ�����Ϊ������̬�ĺ���
void initServosToForwardPose(void) {
	SERVO_SetAngle(1,1,90);   //�Ƕȱ����̧
	SERVO_SetAngle(1,2,75);   //�Ƕȱ�С��̧
	SERVO_SetAngle(1,3,287);	//�Ƕȱ�С��̧
	SERVO_SetAngle(1,4,35);	
}



// TIM1 �����ʼ������
void SERVO_TIM1_Init(void)
{
    /*����ʱ��*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			//����TIM1��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);			//����GPIOE��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    //ʹ�� AFIO ʱ��
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);  //TIM1 �Ĳ�����ӳ��1

    /*GPIO��ʼ��*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);							//��PE9��PE11��PE13��PE14���ų�ʼ��Ϊ�����������

    /*ʱ����Ԫ��ʼ��*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    /*����Ƚϳ�ʼ�� */ 
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // ʹ�������
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // ʹ�ܸ����ж�
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    // ����NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*TIMʹ��*/
    TIM_Cmd(TIM1, ENABLE);
}

// TIM3 �����ʼ������
void SERVO_TIM3_Init(void)
{
    /*����ʱ��*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//����TIM3��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			//����GPIOB��ʱ��

    /*GPIO��ʼ��*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);							//��PB0 - PB1���ų�ʼ��Ϊ�����������

    /*ʱ����Ԫ��ʼ��*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//����ṹ�����
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //������ģʽ��ѡ�����ϼ���
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;				//�������ڣ���ARR��ֵ
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;				//Ԥ��Ƶ������PSC��ֵ
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;               //�ظ����������߼���ʱ���Ż��õ�
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             //���ṹ���������TIM_TimeBaseInit������TIM3��ʱ����Ԫ

    /*����Ƚϳ�ʼ�� */ 
    TIM_OCInitTypeDef TIM_OCInitStructure;							//����ṹ�����
    TIM_OCStructInit(&TIM_OCInitStructure);                         //�ṹ���ʼ��
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //����Ƚ�ģʽ��ѡ��PWMģʽ1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //������ԣ�ѡ��Ϊ��
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //���ʹ��
    TIM_OCInitStructure.TIM_Pulse = 0;								//��ʼ��CCR

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);                        //����TIM3������Ƚ�ͨ��3
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);                        //����TIM3������Ƚ�ͨ��4

    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // ʹ�ܸ����ж�
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // ����NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*TIMʹ��*/
    TIM_Cmd(TIM3, ENABLE);			//ʹ��TIM3����ʱ����ʼ����
}

// TIM4 �����ʼ������
void SERVO_TIM4_Init(void)
{
    /*����ʱ��*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);			//����TIM4��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			//����GPIOB��ʱ��

    /*GPIO��ʼ��*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);							//��PB8 - PB9���ų�ʼ��Ϊ�����������

    /*ʱ����Ԫ��ʼ��*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    /*����Ƚϳ�ʼ�� */ 
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // ʹ�ܸ����ж�
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    // ����NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*TIMʹ��*/
    TIM_Cmd(TIM4, ENABLE);
}





// ����ȫ�ֱ���
typedef struct {
    float current_ccr;
    float target_ccr;
    uint16_t step_count;
} ServoControl;

ServoControl servo_tim1[4]; // TIM1ͨ��1-4
ServoControl servo_tim3[2]; // TIM3ͨ��3-4
ServoControl servo_tim4[1]; // TIM4ͨ��3

volatile uint16_t total_steps = 20; // �ܲ���

// ���Ҳ�ֵ����
float sinusoidal_interpolation(float start, float end, float t) {
    return start + (end - start) * (1 - cosf(t * 3.1415926f)) / 2;
}

// �޸��жϴ�����ʹ�����Ҳ�ֵ
void update_servo_ccr(ServoControl *servo, TIM_TypeDef *TIMx, uint8_t channel) {
    if (servo->step_count < total_steps) {
        float t = (float)servo->step_count / total_steps;
        float value = sinusoidal_interpolation(servo->current_ccr, servo->target_ccr, t);
        switch (channel) {
            case 1: TIM_SetCompare1(TIMx, (uint16_t)value); break;
            case 2: TIM_SetCompare2(TIMx, (uint16_t)value); break;
            case 3: TIM_SetCompare3(TIMx, (uint16_t)value); break;
            case 4: TIM_SetCompare4(TIMx, (uint16_t)value); break;
        }
        servo->step_count++;
    } else {
        servo->step_count = 0;
        servo->current_ccr = servo->target_ccr;
    }
}

// ���ö���Ƕ�
void SERVO_SetAngle(uint8_t timer, uint8_t channel, float Angle)
{
    float compare;

    // ���ݶ�����ͼ���Ƚ�ֵ
    if ((timer == 1 && (channel == 1 || channel == 2))) // 270����
    {
        compare = (Angle / 270 * 2000 + 500);
    }
    else if ((timer == 1 && (channel == 3 || channel == 4)) || (timer == 4 && (channel == 3))) // 360����
    {
        compare = (Angle / 360 * 2000 + 500);
    }
    else if (timer == 3 && (channel == 3 || channel == 4)) // 180����
    {
        compare = (Angle / 180 * 2000 + 500);
    }
    else // Ĭ��180����
    {
        compare = (Angle / 180 * 2000 + 500);
    }

    // ����Ŀ��CCRֵ
    switch (timer)
    {
        case 1:
            if (channel >= 1 && channel <= 4)
            {
                servo_tim1[channel - 1].target_ccr = compare;
                servo_tim1[channel - 1].step_count = 0;
            }
            break;
        case 3:
            if (channel == 3)
            {
                servo_tim3[0].target_ccr = compare;
                servo_tim3[0].step_count = 0;
            }
            else if (channel == 4)
            {
                servo_tim3[1].target_ccr = compare;
                servo_tim3[1].step_count = 0;
            }
            break;
        case 4:
            if (channel == 3)
            {
                servo_tim4[0].target_ccr = compare;
                servo_tim4[0].step_count = 0;
            }
            break;
        default:
            break;
    }
}

//�ж�����
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        for (int i = 0; i < 4; i++)
        {
            update_servo_ccr(&servo_tim1[i], TIM1, i + 1);
        }
        // ����жϱ�־λ
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        for (int i = 0; i < 2; i++)
        {
            update_servo_ccr(&servo_tim3[i], TIM3, i + 3);
        }
        // ����жϱ�־λ
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        update_servo_ccr(&servo_tim4[0], TIM4, 3);
        // ����жϱ�־λ
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}


