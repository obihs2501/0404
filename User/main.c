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


float  Target_SpeedA=0,Target_SpeedB=0,Target_SpeedC=0,Target_SpeedD=0;     //���Ŀ��ֵ
float  Actual_SpeedA=0,Actual_SpeedB,Actual_SpeedC,Actual_SpeedD;
float  Out_A,Out_B,Out_C,Out_D;

float  Actual_LocationA,Actual_LocationB,Actual_LocationC,Actual_LocationD;
float  Target_LocationA=0,Target_LocationB=0,Target_LocationC=0,Target_LocationD=0;

extern float Pwm_Lowerlimit, Pwm_Upperlimit;

extern uint8_t action_executed;//�ñ�־λ�ǵ�����¥������һ�γ���


// ����PS2�ֱ���ر���
int PS2_LX = 0;
int PS2_LY = 0;
int PS2_RX = 0;
int PS2_RY = 0;
int PS2_KEY = 0;
// ģʽ��־λ���������ֲ�ͬ�Ĺ���ģʽ
int mode_flag =1;
/*

	
//����
//���A��PWM��PC6 TURN:PB4 PB5 ���B��PWM��PC7 TURN:PC2 PC3 
//���C��PWM��PC8 TURN:PC4 PC5 ���D��PWM��PC9 TURN:PD7 PD10 
//�������ȷ����˳�����£�
//���A: AIN1��PB4   AIN2:PB5      A�ࣺPA15 B�ࣺPB3 
//���B: BIN1��PC2   BIN2:PC3      A�ࣺPA7  B�ࣺPA6 																			A        B

//���C: CIN1��PC4   CIN2:PC5      A�ࣺPB6 B�ࣺPB7  
//���D: DIN1:PD7    DIN2:PD10      A�ࣺPA1 B�ࣺPA0  																			C        D

//���������ߣ�A�����PA15,PB3 ; B�����PA6 PA7 ;	C�����PB6 PB7  D���:PA0 PA1
//��Ȧ��������Ϊ��11*4*48=2112


���1��PE9
���2��PE11
���3��PE13
���2��PE14
*/  
  
int main(void)
{
    // ϵͳʱ�ӳ�ʼ��
    SystemInit();

    // �����ʱ����ʼ��
    SERVO_TIM1_Init();
    SERVO_TIM3_Init();
    SERVO_TIM4_Init();

    // �������������ö����ʼ�Ƕ�
    SERVO_SetAngle(1, 2, 180);  // ���� TIM1 ͨ�� 1 �Ķ���Ƕ�Ϊ 90 ��

		
		while(1){  
			
//        PS2_KEY = PS2_DataKey();
//        // ��ʱ50ms
//       
//				if (mode_flag == 1)
//        {
//            Manual_Mode();
//        }

 // �������PS2�ֱ���ĳ����������ֵΪ13�����л���ģʽ2
//        if (PS2_KEY == 13)
//        {
//            mode_flag = 2;
//        }

//        // �������PS2�ֱ���ĳ����������ֵΪ15�����л���ģʽ1
//        if (PS2_KEY == 15)
//        {
//            mode_flag = 1;
//        }
//				Delay_ms(50);


//	SERVO_SetAngle(1,2,9);   //�Ƕȱ����̧
//	SERVO_SetAngle(1,2,75);   //�Ƕȱ�С��̧
//	SERVO_SetAngle(1,3,287);	//�Ƕȱ�С��̧
//	SERVO_SetAngle(1,4,35);		//�Ƕȱ����̧
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


















//������̬
//	SERVO_SetAngle(1,1,90);
//	SERVO_SetAngle(1,2,90);	
//	SERVO_SetAngle(1,3,290.7);	
//	SERVO_SetAngle(1,4,36);		
//	
	
	
	
//������̬
//	SERVO_SetAngle(1,1,99);
//	SERVO_SetAngle(1,2,86.4);	
//	SERVO_SetAngle(1,3,29.7);	
//	SERVO_SetAngle(1,4,301.5);	

