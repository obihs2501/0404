#include "stm32f10x.h"
#include "My_math.h"
#include "motor.h"
#include "encoder.h"
#include "Delay.h"
#include "exti.h"
#include "MPU6050.h"
#include "My_PID.h"
#include "Serial.h"
#include "Key.h"
#include "Servo.h"
#include "usart.h"
#include "oled.h"
#include "pstwo.h"

#define STATE_IDLE 0

#define STATE_STEP_1 1
#define STATE_STEP_2 2
#define STATE_STEP_3 3
#define STATE_STEP_4 4

extern uint8_t command_flag;
extern float Distance1 ;
extern float Distance2 ;
int EnocdeA;
extern  uint8_t command;
u8 star_flag = 0;


float  Target_SpeedA=0,Target_SpeedB=0,Target_SpeedC=0,Target_SpeedD=0;     //电机目标值
float  Actual_SpeedA=0,Actual_SpeedB,Actual_SpeedC,Actual_SpeedD;
float  Out_A,Out_B,Out_C,Out_D;

float  Actual_LocationA,Actual_LocationB,Actual_LocationC,Actual_LocationD;
float  Target_LocationA=0,Target_LocationB=0,Target_LocationC=0,Target_LocationD=0;

extern float Pwm_Lowerlimit, Pwm_Upperlimit;

extern uint8_t action_executed;//该标志位是调试上楼动作第一次尝试


// 定义PS2手柄相关变量
int PS2_LX = 0;
int PS2_LY = 0;
int PS2_RX = 0;
int PS2_RY = 0;
int PS2_KEY = 0;
// 模式标志位，用于区分不同的工作模式
int mode_flag =1;
/*

	
//接线
//电机A：PWM：PC6 TURN:PB4 PB5 电机B：PWM：PC7 TURN:PC2 PC3 
//电机C：PWM：PC8 TURN:PC4 PC5 电机D：PWM：PC9 TURN:PD7 PD10 
//电机的正确接线顺序如下：
//电机A: AIN1：PB4   AIN2:PB5      A相：PA15 B相：PB3 
//电机B: BIN1：PC2   BIN2:PC3      A相：PA7  B相：PA6 																			A        B

//电机C: CIN1：PC4   CIN2:PC5      A相：PB6 B相：PB7  
//电机D: DIN1:PD7    DIN2:PD10      A相：PA1 B相：PA0  																			C        D

//编码器接线：A电机：PA15,PB3 ; B电机：PA6 PA7 ;	C电机：PB6 PB7  D电机:PA0 PA1
//以圈的脉冲数为：11*4*48=2112


舵机1：PE9
舵机2：PE11
舵机3：PE13
舵机2：PE14
*/  
  
int main(void)
{
    // 系统时钟初始化
    SystemInit();

    // 舵机定时器初始化
    SERVO_TIM1_Init();
    SERVO_TIM3_Init();
    SERVO_TIM4_Init();

    // 可以在这里设置舵机初始角度
    SERVO_SetAngle(1, 2, 180);  // 设置 TIM1 通道 1 的舵机角度为 90 度

		
		while(1){  
			
//        PS2_KEY = PS2_DataKey();
//        // 延时50ms
//       
//				if (mode_flag == 1)
//        {
//            Manual_Mode();
//        }

 // 如果按下PS2手柄的某个按键（键值为13），切换到模式2
//        if (PS2_KEY == 13)
//        {
//            mode_flag = 2;
//        }

//        // 如果按下PS2手柄的某个按键（键值为15），切换到模式1
//        if (PS2_KEY == 15)
//        {
//            mode_flag = 1;
//        }
//				Delay_ms(50);


//	SERVO_SetAngle(1,2,9);   //角度变大上抬
//	SERVO_SetAngle(1,2,75);   //角度变小上抬
//	SERVO_SetAngle(1,3,287);	//角度变小上抬
//	SERVO_SetAngle(1,4,35);		//角度变大上抬
			}

		
			
	}
	 
	
	
	
//		if (command_flag) 
//			{
//        command_flag = 0;
//        switch (command) 
//					{
//            case 1: Go_Forward(10, 50, 50, 50); break;
//            case 2: Back_Up(20, 50, 50, 50);    break;
//            case 3: Pan_Left(30, 50, 50, 50);   break;
//            case 4: Pan_Right(40, 50, 50, 50); break;
//            case 5: Left_Turn_Self(50, 50, 50, 50); break;
//            case 6: Right_Turn_Self(60, 50, 50, 50); break;
//            default: motor_Stop(); break;
//						}
//				}



void TIM6_IRQHandler(void)
{
		static uint16_t Count;
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
   
					Count++;
						
			if(Count>=50){
					Count=0;
				
					Actual_SpeedA = Encoder_Get1();
					Actual_SpeedB = Encoder_Get2();
					Actual_SpeedC = Encoder_Get3();
					Actual_SpeedD = Encoder_Get4();
				
					Actual_LocationA+=Actual_SpeedA;
					Actual_LocationB+=Actual_SpeedB;					
					Actual_LocationC+=Actual_SpeedC;				
					Actual_LocationD+=Actual_SpeedD;	
					
					
//					Out_A=PositionalPID_SpeedControl_A(Actual_SpeedA,Target_SpeedA);
//					Out_B=PositionalPID_SpeedControl_B(Actual_SpeedB,Target_SpeedB);
//					Out_C=PositionalPID_SpeedControl_C(Actual_SpeedC,Target_SpeedC);
//					Out_D=PositionalPID_SpeedControl_D(Actual_SpeedD,Target_SpeedD);
				
					Out_A=PositionalPID_PositionControl_A(Actual_LocationA,Target_SpeedA);
					Out_B=PositionalPID_PositionControl_B(Actual_LocationB,Target_SpeedB);
					Out_C=PositionalPID_PositionControl_C(Actual_LocationC,Target_SpeedC);
					Out_D=PositionalPID_PositionControl_D(Actual_LocationD,Target_SpeedD);
					
						
//					Set_PWM(Out_A,Out_B,Out_C,Out_D);
					
	
				}
		}
}


















//正向姿态
//	SERVO_SetAngle(1,1,90);
//	SERVO_SetAngle(1,2,90);	
//	SERVO_SetAngle(1,3,290.7);	
//	SERVO_SetAngle(1,4,36);		
//	
	
	
	
//反向姿态
//	SERVO_SetAngle(1,1,99);
//	SERVO_SetAngle(1,2,86.4);	
//	SERVO_SetAngle(1,3,29.7);	
//	SERVO_SetAngle(1,4,301.5);	

