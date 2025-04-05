#ifndef __Servo_H
#define __Servo_H
#include "stm32f10x.h" 

void initServosToForwardPose(void);
extern uint8_t current_state;
void TIM1_UP_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void SERVO_TIM3_Init(void);
void SERVO_TIM1_Init(void);
void SERVO_SetCompare(uint8_t timer, uint8_t channel, uint16_t Compare);
void TransformForwardToReverse(void);
void TransformReverseToForward(void);
void SERVO_SetAngle(uint8_t timer, uint8_t channel, float Angle);
void SERVO_SetAngleSmooth(uint8_t timer, uint8_t channel, float target_angle, int steps);
void Climbing_Action_Perform1(void);

#endif
