#include "stm32f10x.h"
#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "math.h"
#include "My_PID.h"

extern long int Motor_A;
extern long int Motor_B;
extern long int Motor_C;
extern long int Motor_D;        //电机PWM变量
extern int8_t flaga;
extern int8_t flagb;
extern int8_t flagc;
extern int8_t flagd;//标志小车各轮子的方向
extern s16 Get_Data[20];//存放t265的（x,y,z,yaw）


//int16_t  Encoder_A,Encoder_B,Encoder_C,Encoder_D;//电机的转速
u8 count = 0;
u8 pose_flag = 0;
int now_x;int now_y;int targetyaw;
extern u8 star_flag;
//外部中断测电机转速
int x1 = 0;int y1 = 100;
int x2 = 90;int y2 = 100;
int x3 = 180;int y3 = 100;
int x4 = 180;int y4 = 0;
int x5 = 90;int y5 =0;
int x6 = 0;int y6 = 0;

//int EXTI15_10_IRQHandler(void) 
//{    
//	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15) == 0)
//	{
//		EXTI->PR = 1 << 15; //清除LINE5上的中断标志位
//		Encoder_A = Read_Encoder(2);  //===读取编码器的值
//		Encoder_B = Read_Encoder(3);  //===读取编码器的值
//		Encoder_C = -Read_Encoder(4); //===读取编码器的值
//		Encoder_D = -Read_Encoder(5); //===读取编码器的值
//		
////		now_x = Get_Data[0];
////		now_y = Get_Data[1];
////		targetyaw = 0 - Get_Data[3];
//		count = count + 1;
//		if(count == 2 && star_flag == 1)//15ms执行一次电机
//		{
////			drawcircle(40,30);
////			count = 0;
////			if(pose_flag == 0)
////			{
////				My_Cascade_PID(now_x,now_y,x1,y1,targetyaw);
////				Actuator(Motor_A,Motor_B,Motor_C,Motor_D,flaga,flagb,flagc,flagd);
////				count = 0;
////				if(fabs(x1 - now_x) < 4 && fabs(y1 - now_y) < 4)
////				{
////					pose_flag = 1;
////				}
////			}
////			else if(pose_flag == 1)
////			{
////				
////				My_Cascade_PID(now_x,now_y,x2,y2,targetyaw);
////				Actuator(Motor_A,Motor_B,Motor_C,Motor_D,flaga,flagb,flagc,flagd);
////				count = 0;
////				if(fabs(x2 - now_x) < 4 && fabs(y2 - now_y) < 4)
////				{
////					pose_flag = 2;
////				}
////			}
////			else if(pose_flag == 2)
////			{
////				
////				My_Cascade_PID(now_x,now_y,x3,y3,targetyaw);
////				Actuator(Motor_A,Motor_B,Motor_C,Motor_D,flaga,flagb,flagc,flagd);
////				count = 0;
////				if(fabs(x3 - now_x) < 4 && fabs(y3 - now_y) < 4)
////				{
////					pose_flag = 3;
////				}
////			}
////			else if(pose_flag == 3)
////			{
////				
////				My_Cascade_PID(now_x,now_y,x4,y4,targetyaw);
////				Actuator(Motor_A,Motor_B,Motor_C,Motor_D,flaga,flagb,flagc,flagd);
////				count = 0;
////				if(fabs(x4 - now_x) < 4 && fabs(y4 - now_y) < 4)
////				{
////					pose_flag = 4;
////				}
////			}
////			else if(pose_flag == 4)
////			{
////				
////				My_Cascade_PID(now_x,now_y,x5,y5,targetyaw);
////				Actuator(Motor_A,Motor_B,Motor_C,Motor_D,flaga,flagb,flagc,flagd);
////				count = 0;
////				if(fabs(x5 - now_x) < 4 && fabs(y5 - now_y) < 4)
////				{
////					pose_flag = 5;
////				}
////			}
////			else if(pose_flag == 5)
////			{
////				
////				My_Cascade_PID(now_x,now_y,x6,y6,targetyaw);
////				Actuator(Motor_A,Motor_B,Motor_C,Motor_D,flaga,flagb,flagc,flagd);
////				count = 0;
////				if(fabs(x6 - now_x) < 4 && fabs(y6 - now_y) < 4)
////				{
////					pose_flag = 6;
////				}
////			}
////			else
////			{
////				motor_Stop();
////			}
//		}
//	}
//}
       

