#include "stm32f10x.h"
#ifndef __SERIAL_H
#define __SERIAL_H

void Climbing_Action_Perform1(void);
void TransformForwardToReverse(void);
void TransformReverseToForward(void);
void initServosToForwardPose(void);
void SERVO_TIM1_Init(void);
void SERVO_TIM3_Init(void);
void SERVO_TIM4_Init(void);
void SERVO_SetCompare(uint8_t timer, uint8_t channel, uint16_t Compare);
void SERVO_SetAngle(uint8_t timer, uint8_t channel, float Angle);


extern uint8_t action_executed;//�ñ�־λ�ǵ�����¥������һ�γ���
extern uint8_t transform_flag;  // �����־
#endif

