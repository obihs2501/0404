#include "stm32f10x.h"
#include "motor.h"
#include "usart.h"
extern int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;

//初始化控制电机速度的引脚（A电机:B4 B5 ；B电机:C2 C3；C电机: C4 C5；D电机:D7 D10）
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE); //使能PB PC端口时钟
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIOC 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIOC 
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_10;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOD, &GPIO_InitStructure);					      //根据设定参数初始化GPIOC 
		
   //根据设定参数初始化GPIOC 
}

//初始化控制电机速度的引脚（A电机:C6；B电机:C7；C电机:C8；D电机：D电机）
void PWM8_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	Motor_Init(); //控制电机正反转引脚初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);  //使能GPIO外设时钟使能
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //TIM_CH1 //TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 100-1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Prescaler =720-1; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx

	TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能	

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能		
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH4预装载使能	 
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	TIM_Cmd(TIM8, ENABLE);  //使能TIM
 
}


/*****************电机正反转及速度控制*******************/

////小车运动函数 控制四个电机的正反转
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


//前进
void Go_Forward(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd)////输入 速度的绝对值
{
	
	
	Set_PWM(pwma,pwmb,pwmc,pwmd);

}



//后退
void Back_Up(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd)////输入 速度的绝对值
{
	Set_PWM(-pwma,-pwmb,-pwmc,-pwmd);
	 printf("Backing Up\n");
}


////右
void Pan_Right(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd)
{

	Set_PWM(-pwma,pwmb,pwmc,-pwmd);
}


////左
void Pan_Left(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd)
{

	Set_PWM(pwma,-pwmb,-pwmc,pwmd);
}
//小车停止函数

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
	
//		//控制舵机//1换0，交换的意思是不允许乱按，不能连续按1搞坏电机
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

