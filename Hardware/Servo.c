#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Servo.h" 
#include <math.h>
#include <stdlib.h>
// 定义状态
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

//反向姿态
//	SERVO_SetAngle(1,1,99);
//	SERVO_SetAngle(1,2,86.4);	
//	SERVO_SetAngle(1,3,29.7);	
//	SERVO_SetAngle(1,4,301.5);

//正向姿态
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
            // 执行第1个动作
						SERVO_SetAngle(1, 4, 36);
						SERVO_SetAngle(1, 2, 90);
				    SERVO_SetAngle(1, 3, 290);
						SERVO_SetAngle(1, 1, 84);
            Delay_ms(2000);
            current_state = STATE_ACTION_2;
            // 注意这里没有 break，继续执行下一个 case
        case STATE_ACTION_2:
            // 执行第2个动作
            SERVO_SetAngle(1, 4, 54);
				    Delay_ms(1000);
						SERVO_SetAngle(1, 2, 48);
				    SERVO_SetAngle(1, 3, 290.7);
						SERVO_SetAngle(1, 1, 90);
            Delay_ms(2000);
            current_state = STATE_ACTION_3;
        case STATE_ACTION_3:
            // 执行第3个动作
            SERVO_SetAngle(1, 4, 108);
				    Delay_ms(1500);
						SERVO_SetAngle(1, 2, 167);
            SERVO_SetAngle(1, 3, 290.7);
						SERVO_SetAngle(1, 1, 90);				    
				    current_state = STATE_ACTION_4;
				case STATE_ACTION_4:
            // 执行第4个动作
            SERVO_SetAngle(1, 4, 108);			    
						SERVO_SetAngle(1, 2, 167);
            SERVO_SetAngle(1, 3, 290.7);
						SERVO_SetAngle(1, 1, 90);
            Delay_ms(2000);
            current_state = STATE_ACTION_5;
				case STATE_ACTION_5:
            // 执行第5个动作
            SERVO_SetAngle(1, 4, 108);
						SERVO_SetAngle(1, 2, 167);				 
    				SERVO_SetAngle(1, 3, 254.7);
						SERVO_SetAngle(1, 1, 138);
            Delay_ms(2000);
            current_state = STATE_ACTION_6;
				case STATE_ACTION_6:
            // 执行第6个动作
            SERVO_SetAngle(1, 4, 108);		    
						SERVO_SetAngle(1, 2, 167);
            SERVO_SetAngle(1, 3, 218.7);
				    Delay_ms(2000);
						SERVO_SetAngle(1, 1, 36);		
//            current_state = STATE_ACTION_7;
//				case STATE_ACTION_7:
//            //动作7：电机控制前进
//				    Delay_ms(2000);
//				    Delay_ms(2000);
//            current_state = STATE_ACTION_8;
//				case STATE_ACTION_8:
//					// 执行第8个动作
//				    SERVO_SetAngle(1, 4, 108);
//            SERVO_SetAngle(1, 3, 218.7);	
//				    Delay_ms(2000);				
// 						SERVO_SetAngle(1, 1, 40);
//						SERVO_SetAngle(1, 2, 163);
//				    Delay_ms(2000);
//            current_state = STATE_ACTION_9;
//				case STATE_ACTION_9:
//					// 执行第9个动作
// 						SERVO_SetAngle(1, 1, 108);
//						SERVO_SetAngle(1, 2, 75);
// 				    Delay_ms(2000);	   				
//				    SERVO_SetAngle(1, 4, 36);
//            SERVO_SetAngle(1, 3, 290);	
//				    current_state = STATE_ACTION_10;
//					case STATE_ACTION_10:
//					// 执行第10个动作
//				    SERVO_SetAngle(1, 4, 36);
//            SERVO_SetAngle(1, 3, 290);
// 				    Delay_ms(2000);	   				
//					  SERVO_SetAngle(1, 1, 84);
//						SERVO_SetAngle(1, 2, 90);		
						action_executed = 0;//注意初始化是1，做完最后一个动作置0，若想再次使用这套动作可通过将标志位复位
            break;
        default:
            break;
    }
}


 uint8_t transform_flag;  // 清除标志

void TransformForwardToReverse(void)
{
    // 设置反向姿态
      SERVO_SetAngle(1, 1, 99);
      SERVO_SetAngle(1, 3, 29.7);	   
     	SERVO_SetAngle(1, 2, 90);
      SERVO_SetAngle(1, 4, 36);  
		    Delay_ms(2000);
      SERVO_SetAngle(1, 1, 99);
      SERVO_SetAngle(1, 3, 29.7);	
	    SERVO_SetAngle(1, 2, 86.4);
      SERVO_SetAngle(1, 4, 301.5);   		  

	    transform_flag = 0;  // 清除标志
}

// 反向到正向姿态变换函数
void TransformReverseToForward(void)
{
    // 设置正向姿态
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


// 初始化舵机为正向姿态的函数
void initServosToForwardPose(void) {
	SERVO_SetAngle(1,1,90);   //角度变大上抬
	SERVO_SetAngle(1,2,75);   //角度变小上抬
	SERVO_SetAngle(1,3,287);	//角度变小上抬
	SERVO_SetAngle(1,4,35);	
}



// TIM1 舵机初始化函数
void SERVO_TIM1_Init(void)
{
    /*开启时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			//开启TIM1的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);			//开启GPIOE的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    //使能 AFIO 时钟
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);  //TIM1 的部分重映射1

    /*GPIO初始化*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);							//将PE9、PE11、PE13、PE14引脚初始化为复用推挽输出

    /*时基单元初始化*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    /*输出比较初始化 */ 
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

    // 使能主输出
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // 使能更新中断
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    // 配置NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*TIM使能*/
    TIM_Cmd(TIM1, ENABLE);
}

// TIM3 舵机初始化函数
void SERVO_TIM3_Init(void)
{
    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//开启TIM3的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			//开启GPIOB的时钟

    /*GPIO初始化*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);							//将PB0 - PB1引脚初始化为复用推挽输出

    /*时基单元初始化*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;				//计数周期，即ARR的值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;				//预分频器，即PSC的值
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;               //重复计数器，高级定时器才会用到
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元

    /*输出比较初始化 */ 
    TIM_OCInitTypeDef TIM_OCInitStructure;							//定义结构体变量
    TIM_OCStructInit(&TIM_OCInitStructure);                         //结构体初始化
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //输出比较模式，选择PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //输出极性，选择为高
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;								//初始的CCR

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);                        //配置TIM3的输出比较通道3
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);                        //配置TIM3的输出比较通道4

    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 使能更新中断
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // 配置NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*TIM使能*/
    TIM_Cmd(TIM3, ENABLE);			//使能TIM3，定时器开始运行
}

// TIM4 舵机初始化函数
void SERVO_TIM4_Init(void)
{
    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);			//开启TIM4的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			//开启GPIOB的时钟

    /*GPIO初始化*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);							//将PB8 - PB9引脚初始化为复用推挽输出

    /*时基单元初始化*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    /*输出比较初始化 */ 
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // 使能更新中断
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    // 配置NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*TIM使能*/
    TIM_Cmd(TIM4, ENABLE);
}





// 定义全局变量
typedef struct {
    float current_ccr;
    float target_ccr;
    uint16_t step_count;
} ServoControl;

ServoControl servo_tim1[4]; // TIM1通道1-4
ServoControl servo_tim3[2]; // TIM3通道3-4
ServoControl servo_tim4[1]; // TIM4通道3

volatile uint16_t total_steps = 20; // 总步数

// 正弦插值函数
float sinusoidal_interpolation(float start, float end, float t) {
    return start + (end - start) * (1 - cosf(t * 3.1415926f)) / 2;
}

// 修改中断处理函数使用正弦插值
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

// 设置舵机角度
void SERVO_SetAngle(uint8_t timer, uint8_t channel, float Angle)
{
    float compare;

    // 根据舵机类型计算比较值
    if ((timer == 1 && (channel == 1 || channel == 2))) // 270°舵机
    {
        compare = (Angle / 270 * 2000 + 500);
    }
    else if ((timer == 1 && (channel == 3 || channel == 4)) || (timer == 4 && (channel == 3))) // 360°舵机
    {
        compare = (Angle / 360 * 2000 + 500);
    }
    else if (timer == 3 && (channel == 3 || channel == 4)) // 180°舵机
    {
        compare = (Angle / 180 * 2000 + 500);
    }
    else // 默认180°舵机
    {
        compare = (Angle / 180 * 2000 + 500);
    }

    // 设置目标CCR值
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

//中断设置
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        for (int i = 0; i < 4; i++)
        {
            update_servo_ccr(&servo_tim1[i], TIM1, i + 1);
        }
        // 清除中断标志位
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
        // 清除中断标志位
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        update_servo_ccr(&servo_tim4[0], TIM4, 3);
        // 清除中断标志位
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}


