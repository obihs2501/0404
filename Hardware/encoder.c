#include "stm32f10x.h"
#include "encoder.h"

/**************************************************************************

**************************************************************************/
void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// 需要使能AFIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器2的时钟
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);//引脚重映射
	

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PA端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //浮空输入    
	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB 
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_ICStructInit(&TIM_ICInitStructure);    //给默认的初始值
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;         //通道一
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                //滤波大小 
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;         //通道一
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                //滤波大小  
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_Cmd(TIM2, ENABLE); 
}
/**************************************************************************

**************************************************************************/
void Encoder_Init_TIM3(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//使能定时器3的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PA端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA
  
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 1-1; // 预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


	
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	//定时器编码器接口配置  //通道不反相
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	
	
	TIM_Cmd(TIM3, ENABLE); 
}

/**************************************************************************

**************************************************************************/
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
  
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
  TIM_ICStructInit(&TIM_ICInitStructure);    //给默认的初始值
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;         //通道一
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                //滤波大小 
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;         //通道一
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                //滤波大小  
	TIM_ICInit(TIM4, &TIM_ICInitStructure);	
	

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	TIM_Cmd(TIM4, ENABLE); 
}

/**************************************************************************

**************************************************************************/
void Encoder_Init_TIM5(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);//使能定时器的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PA端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA
  
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	
	
  TIM_ICStructInit(&TIM_ICInitStructure);    //给默认的初始值
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;         //通道一
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                //滤波大小 
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
    
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;         //通道一
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                //滤波大小  
	TIM_ICInit(TIM5, &TIM_ICInitStructure);	
	

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	
	TIM_Cmd(TIM5, ENABLE); 
}
/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
**************************************************************************/



int16_t Encoder_Get1(void)
{
	
	int16_t Temp;
	Temp = (short)TIM_GetCounter(TIM2);
	TIM_SetCounter(TIM2, 0);
	return Temp;
	
}

int16_t Encoder_Get2(void)
{
	
	int16_t Temp;
	Temp = (short)TIM_GetCounter(TIM3);
	TIM_SetCounter(TIM3, 0);
	return Temp;	
}

int16_t Encoder_Get3(void)
{
	
	int16_t Temp;
	Temp = (short)TIM_GetCounter(TIM4);
	TIM_SetCounter(TIM4, 0);
	return Temp;
}


int16_t Encoder_Get4(void)
{
	
	int16_t Temp;
	Temp = (short)TIM_GetCounter(TIM5);
	TIM_SetCounter(TIM5, 0);
	return Temp;
	
}


