#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"
/******控制电机的速度*******/
#define PWMD   TIM8->CCR4  
#define PWMC   TIM8->CCR3  
#define PWMB   TIM8->CCR2 
#define PWMA   TIM8->CCR1 

void PWM8_Init(void);
void Motor_Init(void);

void Set_PWM(int8_t speed1,int8_t speed2,int8_t speed3,int8_t speed4); 


void Go_Forward(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd);
void Back_Up(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd);
void Pan_Right(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd);
void Pan_Left(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd);
void Left_Turn_Self(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd);
void Right_Turn_Self(int8_t pwma,int8_t pwmb,int8_t pwmc,int8_t pwmd);

void motor_Stop(void);

void Manual_Mode(void);


#endif

