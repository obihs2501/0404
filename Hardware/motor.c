#include "stm32f10x.h"
#include "motor.h"
#include "usart.h"
extern int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;

//��ʼ�����Ƶ���ٶȵ����ţ�A���:B4 B5 ��B���:C2 C3��C���: C4 C5��D���:D7 D10��
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE); //ʹ��PB PC�˿�ʱ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOC, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOC 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOC, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOC 
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_10;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOD, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOC 
		
   //�����趨������ʼ��GPIOC 
}

//��ʼ�����Ƶ���ٶȵ����ţ�A���:C6��B���:C7��C���:C8��D�����D�����
void PWM8_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	Motor_Init(); //���Ƶ������ת���ų�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);  //ʹ��GPIO����ʱ��ʹ��
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //TIM_CH1 //TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 100-1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Prescaler =720-1; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE �����ʹ��	

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��	
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��	
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��		
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH4Ԥװ��ʹ��	 
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM
 
}


/*****************�������ת���ٶȿ���*******************/

////С���˶����� �����ĸ����������ת
void Set_PWM(int8_t speed1,int8_t speed2,int8_t speed3,int8_t speed4)       
{
    if(speed1>=0)
    {
		PWMA = speed1;
		GPIO_SetBits(GPIOB, GPIO_Pin_4);
		GPIO_ResetBits(GPIOB, GPIO_Pin_5); 
    }
    else
    {
		PWMA = -speed1;       
		GPIO_ResetBits(GPIOB, GPIO_Pin_4);
    GPIO_SetBits(GPIOB, GPIO_Pin_5);
    }
		
		
	  if(speed2>=0)
    {
    PWMB = speed2;     
   		GPIO_SetBits(GPIOC, GPIO_Pin_2);
			GPIO_ResetBits(GPIOC, GPIO_Pin_3);
    }
    else
    {
		PWMB = -speed2;     
		GPIO_ResetBits(GPIOC, GPIO_Pin_2);
    GPIO_SetBits(GPIOC, GPIO_Pin_3);

    }
		
		
    if(speed3>=0)
    {
    PWMC = speed3;       
    GPIO_ResetBits(GPIOC, GPIO_Pin_4);
    GPIO_SetBits(GPIOC, GPIO_Pin_5);
    }
    else
    {
		PWMC = -speed3;       
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
    }		
		
    if(speed4>=0)
    {
    PWMD = speed4;    
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);
    GPIO_SetBits(GPIOD, GPIO_Pin_10);
    }
    else
    {
		PWMD = -speed4;      
		GPIO_SetBits(GPIOD, GPIO_Pin_7);
		GPIO_ResetBits(GPIOD, GPIO_Pin_10);
    }		
		
		
}


//ǰ��
void Go_Forward(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd)////���� �ٶȵľ���ֵ
{
	
	
	Set_PWM(pwma,pwmb,pwmc,pwmd);

}



//����
void Back_Up(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd)////���� �ٶȵľ���ֵ
{
	Set_PWM(-pwma,-pwmb,-pwmc,-pwmd);
	 printf("Backing Up\n");
}


////��
void Pan_Right(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd)
{

	Set_PWM(-pwma,pwmb,pwmc,-pwmd);
}


////��
void Pan_Left(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd)
{

	Set_PWM(pwma,-pwmb,-pwmc,pwmd);
}
//С��ֹͣ����

void motor_Stop(void)
{
	 Set_PWM(0,0,0,0);
}

void Left_Turn_Self(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd){
		
		Set_PWM(-pwma,pwmb,-pwmc,pwmd);
}
void Right_Turn_Self(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd){
		
		Set_PWM(pwma,-pwmb,pwmc,-pwmd);
}



void Manual_Mode(void)
{
	
  Delay_ms(5);

	if(PS2_KEY==0){
	
		motor_Stop();
	}
	else if(PS2_KEY==5)
	{
		Go_Forward(50,50,50,50);
		 
	}
	else if(PS2_KEY==7)
	{
		Back_Up(50,50,50,50);
		
	}else if(PS2_KEY==16){
	
		Pan_Right(50,50,50,50);
	}	else if(PS2_KEY==14){
	
		Pan_Left(50,50,50,50);
	}
		else if(PS2_KEY==8)
	{
		Left_Turn_Self(50,50,50,50);
		
	}
	else if(PS2_KEY==6)
	{
		Right_Turn_Self(50,50,50,50);
		
	}
	else
	{
		motor_Stop();
	}
	
//		//���ƶ��//1��0����������˼�ǲ������Ұ�������������1�㻵���
//	if(PS2_KEY==11)
//	{

//		
//	}else if
//	(PS2_KEY==9)
//	{

//	}
//	
//	if(PS2_KEY==12)
//	{

//	}else if
//	(PS2_KEY==10)
//	{

//	}
//	
//	if(PS2_KEY==5)
//	{	

//	}else if(PS2_KEY==7)
//	{

//	}
	
}

